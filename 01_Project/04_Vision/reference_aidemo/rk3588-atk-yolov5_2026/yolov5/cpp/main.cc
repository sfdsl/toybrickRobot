/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "yolov5.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <deque>
#include <map>
#include <condition_variable>

/*-------------------------------------------
         Touch Close Button (touchscreen)
-------------------------------------------*/
// 全屏模式下 GTK 标题栏（含关闭按钮）被隐藏，触摸屏无键盘无法点关闭；
// 这里在画面右上角画一个红色 “X” 触摸按钮，点击该区域即退出程序。
static std::atomic<bool> g_quit{false};
static cv::Rect g_btn;   // 触摸关闭按钮在“图像坐标”中的矩形（由主循环每帧填充）

static void on_mouse(int event, int x, int y, int flags, void *userdata)
{
    (void)flags;
    (void)userdata;
    if (event != cv::EVENT_LBUTTONDOWN)
        return;
    // OpenCV 鼠标回调的 (x,y) 本身就是“图像像素坐标”，直接判定按钮矩形即可。
    if (!g_btn.empty()) {
        cv::Rect hit(g_btn.x - 20, g_btn.y - 20, g_btn.width + 40, g_btn.height + 40);
        if (hit.contains(cv::Point(x, y))) {
            g_quit = true;
        }
    }
}

/*-------------------------------------------
                3-Core pipeline types
-------------------------------------------*/
static constexpr int N_CORES = 3;

struct Job {
    int seq;
    cv::Mat frame;   // BGR
};
struct Result {
    int seq;
    cv::Mat frame;   // 已画框的 RGB 帧
};

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc <= 2 || argc > 3)
    {
        printf("%s <model_path> [camera_index]\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    int camera_index = 44; // default value
    if (argc == 3) {
        camera_index = atoi(argv[2]);
    }

    bool use_gui = (getenv("DISPLAY") != nullptr);
    if (!use_gui) {
        printf("[info] 无 DISPLAY，以命令行模式运行（3 核 NPU 并行，每秒打印吞吐 FPS 与 NPU 均耗）\n");
    }

    int ret;
    // 3 个独立 rknn 上下文，分别绑定 NPU 核 0/1/2
    rknn_app_context_t ctx[N_CORES];
    for (int i = 0; i < N_CORES; i++) {
        memset(&ctx[i], 0, sizeof(rknn_app_context_t));
    }
    ret = init_post_process();
    if (ret != 0) {
        printf("init_post_process failed! 请检查 model/bug_labels_list.txt 是否存在。\n");
        return -1;
    }

    const rknn_core_mask core_mask[N_CORES] = {
        RKNN_NPU_CORE_0, (rknn_core_mask)2, (rknn_core_mask)4  // CORE_0=1, CORE_1=2, CORE_2=4
    };
    for (int i = 0; i < N_CORES; i++) {
        ret = init_yolov5_model(model_path, &ctx[i]);
        if (ret != 0) {
            printf("init_yolov5_model[%d] fail! ret=%d model_path=%s\n", i, ret, model_path);
            return -1;
        }
        rknn_set_core_mask(ctx[i].rknn_ctx, core_mask[i]);
        printf("[info] NPU ctx[%d] 绑定核 %d\n", i, i);
    }

    cv::VideoCapture cap(camera_index);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open the camera." << std::endl;
        return -1;
    }

    // 帧队列（cap→workers），有界，满则丢弃最旧帧避免堆积
    std::deque<Job> job_q;
    std::mutex job_mtx;
    std::condition_variable job_cv;
    const size_t MAX_JOBS = 8;

    // 结果集合（workers→main），按 seq 有序取用
    std::map<int, Result> results;
    std::mutex res_mtx;
    std::condition_variable res_cv;
    const size_t MAX_RESULTS = 8;

    std::atomic<bool> running(true);
    std::atomic<int>  g_processed(0);     // worker 完成计数（吞吐统计）
    std::atomic<double> g_acc_npu(0.0);   // NPU 推理累计(ms)
    std::atomic<int>    g_npu_cnt(0);

    // 每个 worker 的预分配 letterbox 输出缓冲
    unsigned char *dst_buf[N_CORES] = {nullptr};
    for (int i = 0; i < N_CORES; i++) {
        image_buffer_t tmp;
        memset(&tmp, 0, sizeof(image_buffer_t));
        tmp.width = ctx[i].model_width;
        tmp.height = ctx[i].model_height;
        tmp.format = IMAGE_FORMAT_RGB888;
        tmp.size = get_image_size(&tmp);
        dst_buf[i] = (unsigned char *)malloc(tmp.size);
        if (!dst_buf[i]) { printf("malloc dst[%d] fail\n", i); return -1; }
    }

    // cap 线程：持续抓帧入队
    // YOLO_BENCH 模式：只抓 1 帧，然后循环喂同一帧，绕开摄像头 ~30fps 上限，
    // 测 NPU 真实吞吐（单核 vs 三核对比用）。
    const char *bench = getenv("YOLO_BENCH");
    std::thread cap_thread([&]() {
        int seq = 0;
        cv::Mat first;
        if (bench) {
            while (first.empty()) {
                if (!cap.read(first)) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); continue; }
            }
            printf("[bench] 已抓取 1 帧，循环喂帧测 NPU 吞吐\n");
        }
        cv::Mat f;
        while (running.load(std::memory_order_relaxed)) {
            if (bench) {
                f = first.clone();
            } else {
                if (!cap.read(f)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
            }
            Job j;
            j.seq = seq++;
            f.copyTo(j.frame);
            {
                std::lock_guard<std::mutex> lk(job_mtx);
                if (job_q.size() >= MAX_JOBS) {
                    job_q.pop_front();
                }
                job_q.push_back(std::move(j));
            }
            job_cv.notify_one();
        }
    });

    // 3 个 worker 线程：各自 ctx/core，拉取任务→letterbox→推理→后处理→画框→回传结果
    std::thread workers[N_CORES];
    for (int wid = 0; wid < N_CORES; wid++) {
        workers[wid] = std::thread([&, wid]() {
            image_buffer_t src_image;
            image_buffer_t dst_img;
            memset(&dst_img, 0, sizeof(image_buffer_t));
            dst_img.width = ctx[wid].model_width;
            dst_img.height = ctx[wid].model_height;
            dst_img.format = IMAGE_FORMAT_RGB888;
            dst_img.size = get_image_size(&dst_img);
            dst_img.virt_addr = dst_buf[wid];

            rknn_input inputs[ctx[wid].io_num.n_input];
            rknn_output outputs[ctx[wid].io_num.n_output];
            const float nms_threshold = NMS_THRESH;
            const float box_conf_threshold = BOX_THRESH;
            int bg_color = 114;

            while (running.load(std::memory_order_relaxed)) {
                Job j;
                {
                    std::unique_lock<std::mutex> lk(job_mtx);
                    job_cv.wait(lk, [&]{ return !running.load(std::memory_order_relaxed) || !job_q.empty(); });
                    if (!running.load(std::memory_order_relaxed) && job_q.empty()) break;
                    if (job_q.empty()) continue;
                    j = std::move(job_q.front());
                    job_q.pop_front();
                }
                job_cv.notify_one();

                cv::Mat frame = j.frame;
                // BGR→RGB（模型需要 RGB）
                cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

                memset(&src_image, 0, sizeof(image_buffer_t));
                src_image.height = frame.rows;
                src_image.width = frame.cols;
                src_image.width_stride = frame.step[0];
                src_image.virt_addr = frame.data;
                src_image.format = IMAGE_FORMAT_RGB888;
                src_image.size = frame.total() * frame.elemSize();

                letterbox_t letter_box;
                memset(&letter_box, 0, sizeof(letterbox_t));
                memset(inputs, 0, sizeof(inputs));
                memset(outputs, 0, sizeof(outputs));
                object_detect_result_list od_results;
                memset(&od_results, 0, sizeof(od_results));

                // letterbox（dst 预分配复用）
                ret = convert_image_with_letterbox(&src_image, &dst_img, &letter_box, bg_color);
                if (ret < 0) {
                    continue;
                }

                inputs[0].index = 0;
                inputs[0].type = RKNN_TENSOR_UINT8;
                inputs[0].fmt = RKNN_TENSOR_NHWC;
                inputs[0].size = ctx[wid].model_width * ctx[wid].model_height * ctx[wid].model_channel;
                inputs[0].buf = dst_img.virt_addr;
                rknn_inputs_set(ctx[wid].rknn_ctx, ctx[wid].io_num.n_input, inputs);

                auto t0 = std::chrono::steady_clock::now();
                rknn_run(ctx[wid].rknn_ctx, nullptr);

                for (int i = 0; i < ctx[wid].io_num.n_output; i++) {
                    outputs[i].index = i;
                    outputs[i].want_float = (!ctx[wid].is_quant);
                }
                rknn_outputs_get(ctx[wid].rknn_ctx, ctx[wid].io_num.n_output, outputs, NULL);
                auto t1 = std::chrono::steady_clock::now();
                double npu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                g_acc_npu.store(g_acc_npu.load(std::memory_order_relaxed) + npu_ms, std::memory_order_relaxed);
                g_npu_cnt.fetch_add(1, std::memory_order_relaxed);

                post_process(&ctx[wid], outputs, &letter_box, box_conf_threshold, nms_threshold, &od_results);
                rknn_outputs_release(ctx[wid].rknn_ctx, ctx[wid].io_num.n_output, outputs);

                // 画框（在 RGB 帧上）
                char text[256];
                for (int i = 0; i < od_results.count; i++) {
                    object_detect_result *det = &(od_results.results[i]);
                    int x1 = det->box.left, y1 = det->box.top, x2 = det->box.right, y2 = det->box.bottom;
                    draw_rectangle(&src_image, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);
                    sprintf(text, "%s %.1f%%", coco_cls_to_name(det->cls_id), det->prop * 100);
                    draw_text(&src_image, text, x1, y1 - 20, COLOR_GREEN, 10);
                }

                Result r;
                r.seq = j.seq;
                r.frame = frame;   // 已画框的 RGB 帧
                {
                    std::lock_guard<std::mutex> lk(res_mtx);
                    // 若结果集合过大（main 跟不上），丢弃最旧未取结果，避免无限堆积
                    while (results.size() >= MAX_RESULTS) {
                        results.erase(results.begin());
                    }
                    results.emplace(r.seq, std::move(r));
                }
                res_cv.notify_one();
                g_processed++;
            }
        });
    }

    if (use_gui) {
        cv::namedWindow("out", cv::WINDOW_NORMAL);
        cv::setMouseCallback("out", on_mouse, nullptr);
        cv::setWindowProperty("out", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    }

    // main：取结果绘制/统计。不强制严格顺序，来一个取一个，避免反压 worker 导致
    // 3 核退化成单核（worker 完成顺序不确定时，严格有序会让 main 卡等、结果堆积反压）。
    auto prev_t = std::chrono::steady_clock::now();
    int shown = 0;
    double acc_npu_local = 0.0;
    int npu_local = 0;
    while (true) {
        if (g_quit.load(std::memory_order_relaxed)) break;
        Result r;
        {
            std::unique_lock<std::mutex> lk(res_mtx);
            res_cv.wait(lk, [&]{ return !running.load(std::memory_order_relaxed) || !results.empty(); });
            if (results.empty()) continue;
            auto it = results.begin();
            r = std::move(it->second);
            results.erase(it);
        }

        // 帧率
        auto cur_t = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(cur_t - prev_t).count();
        prev_t = cur_t;
        float fps = (dt > 0.0f) ? (1.0f / dt) : 0.0f;
        char fps_text[64];
        snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", fps);

        shown++;
        // 取一次累计 NPU 均耗快照（用于显示）
        int pc = g_processed.load();
        double an = g_acc_npu.load();
        int nc = g_npu_cnt.load();
        acc_npu_local = an;
        npu_local = nc;

        if (use_gui) {
            // 在帧上画 FPS 与红 X（cv 绘制，避免依赖 image_drawing 的 buffer 校验）
            cv::putText(r.frame, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            {
                int bw = 60, bh = 60;
                int bx = r.frame.cols - bw - 10;
                int by = 10;
                cv::rectangle(r.frame, cv::Rect(bx, by, bw, bh), cv::Scalar(0, 0, 255), 4);
                cv::line(r.frame, cv::Point(bx + 14, by + 14), cv::Point(bx + bw - 14, by + bh - 14), cv::Scalar(0, 0, 255), 5);
                cv::line(r.frame, cv::Point(bx + bw - 14, by + 14), cv::Point(bx + 14, by + bh - 14), cv::Scalar(0, 0, 255), 5);
                g_btn = cv::Rect(bx, by, bw, bh);
            }
            // r.frame 是 RGB，imshow 需 BGR
            cv::Mat bgr;
            cv::cvtColor(r.frame, bgr, cv::COLOR_RGB2BGR);
            cv::imshow("out", bgr);
            int key = cv::waitKey(1);
            if (key == 'q' || key == 27) break;
        } else {
            // 命令行模式：每秒打印一次吞吐 FPS 与 NPU 均耗
            if (shown % 30 == 0) {
                double avg_npu = (npu_local > 0) ? (acc_npu_local / npu_local) : 0.0;
                printf("YOLOv5(3core) FPS: %.1f | npu=%.1fms | processed=%d\n", fps, avg_npu, pc);
            }
        }
    }

    running = false;
    job_cv.notify_all();
    res_cv.notify_all();
    cap_thread.join();
    for (int i = 0; i < N_CORES; i++) workers[i].join();
    for (int i = 0; i < N_CORES; i++) free(dst_buf[i]);

    deinit_post_process();
    for (int i = 0; i < N_CORES; i++) release_yolov5_model(&ctx[i]);

    return 0;
}
