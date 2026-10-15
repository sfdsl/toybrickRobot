// rknn_infer_node.cpp
// 《13_视觉识别实验方案》P2：YOLO11 NPU 推理 ROS2 节点
//   /image_raw (QoS depth=1) → 有界队列（丢旧帧）→ 3 worker 各绑 1 个 NPU 核
//   → letterbox → rknn 推理 → post_process(NMS) → /detections
// 复用 demo 的 postprocess.cc / yolo11.cc / imageutils（单一来源，不复制代码）。
// /detections 格式（std_msgs/Float32MultiArray，方案 §5.3）：
//   layout.dim[0].size = N（目标数），dim[1].size = 6
//   data = [cls, conf, x1, y1, x2, y2] × N（原图像素坐标，COCO 80 类索引）
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "yolo11.h"       // 先：定义 rknn_app_context_t（其内部已 include postprocess.h）
#include "postprocess.h"

namespace {

constexpr int N_CORES = 3;
constexpr int MAX_OUT_TENSORS = 16;   // yolo11 实际 9 输出

struct Job {
  cv::Mat frame;   // BGR，浅拷贝（引用计数），无额外像素拷贝
};

}  // namespace

class RknnInfer : public rclcpp::Node {
public:
  RknnInfer() : Node("rknn_infer") {
    model_path_ = declare_parameter<std::string>("model_path", "model/yolo11.rknn");
    conf_th_    = declare_parameter<double>("conf_threshold", 0.25);
    nms_th_     = declare_parameter<double>("nms_threshold", 0.45);
    max_jobs_   = declare_parameter<int>("queue_size", 8);
    hud_enable_ = declare_parameter<bool>("hud_enable", true);
    hud_fps_    = declare_parameter<double>("hud_fps", 10.0);
    hud_quality_ = declare_parameter<int>("hud_quality", 70);
    if (hud_fps_ > 0) hud_interval_ = std::chrono::milliseconds(int(1000.0 / hud_fps_));

    init_post_process();   // 读 ./model/coco_80_labels_list.txt（main 已 chdir 到 share 目录）

    const rknn_core_mask core_mask[N_CORES] = {
        RKNN_NPU_CORE_0, (rknn_core_mask)2, (rknn_core_mask)4};
    for (int i = 0; i < N_CORES; i++) {
      std::memset(&ctx_[i], 0, sizeof(rknn_app_context_t));
      if (init_yolo11_model(model_path_.c_str(), &ctx_[i]) != 0) {
        RCLCPP_FATAL(get_logger(), "init_yolo11_model[%d] 失败: %s", i, model_path_.c_str());
        throw std::runtime_error("model init failed");
      }
      rknn_set_core_mask(ctx_[i].rknn_ctx, core_mask[i]);
      if (ctx_[i].io_num.n_output > MAX_OUT_TENSORS || ctx_[i].io_num.n_input != 1) {
        RCLCPP_FATAL(get_logger(), "模型 IO 数不符合预期 (in=%u out=%u)",
                     ctx_[i].io_num.n_input, ctx_[i].io_num.n_output);
        throw std::runtime_error("unexpected model IO");
      }
      RCLCPP_INFO(get_logger(), "NPU ctx[%d] 绑定核 %d（模型 %dx%d）",
                  i, i, ctx_[i].model_width, ctx_[i].model_height);
    }

    // 每个 worker 预分配 letterbox 目标缓冲
    for (int i = 0; i < N_CORES; i++) {
      image_buffer_t tmp;
      std::memset(&tmp, 0, sizeof(tmp));
      tmp.width = ctx_[i].model_width;
      tmp.height = ctx_[i].model_height;
      tmp.format = IMAGE_FORMAT_RGB888;
      tmp.size = get_image_size(&tmp);
      dst_buf_[i] = static_cast<unsigned char *>(malloc(tmp.size));
    }

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("detections", 1);
    pub_hud_ = create_publisher<sensor_msgs::msg::CompressedImage>("image_det", 1);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 1, [this](sensor_msgs::msg::Image::SharedPtr msg) { on_image(msg); });

    for (int i = 0; i < N_CORES; i++) workers_.emplace_back([this, i] { worker_loop(i); });
    stats_timer_ = create_wall_timer(std::chrono::seconds(5), [this] { print_stats(); });

    RCLCPP_INFO(get_logger(), "rknn_infer 启动: /image_raw -> %s -> /detections",
                model_path_.c_str());
  }

  ~RknnInfer() override {
    running_ = false;
    cv_.notify_all();
    for (auto &t : workers_) if (t.joinable()) t.join();
    for (int i = 0; i < N_CORES; i++) {
      release_yolo11_model(&ctx_[i]);
      free(dst_buf_[i]);
    }
    deinit_post_process();
  }

private:
  void on_image(sensor_msgs::msg::Image::SharedPtr msg) {
    if (!running_) return;
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;   // 需要独立 buffer（跨线程）
    {
      std::lock_guard<std::mutex> lk(job_mtx_);
      if (job_q_.size() >= static_cast<size_t>(max_jobs_)) job_q_.pop_front();  // 丢旧帧
      job_q_.push_back({frame});
    }
    cv_.notify_one();
  }

  void worker_loop(int wid) {
    image_buffer_t dst_img;
    std::memset(&dst_img, 0, sizeof(dst_img));
    dst_img.width = ctx_[wid].model_width;
    dst_img.height = ctx_[wid].model_height;
    dst_img.format = IMAGE_FORMAT_RGB888;
    dst_img.size = get_image_size(&dst_img);
    dst_img.virt_addr = dst_buf_[wid];

    while (running_) {
      cv::Mat frame;
      {
        std::unique_lock<std::mutex> lk(job_mtx_);
        cv_.wait(lk, [&] { return !running_ || !job_q_.empty(); });
        if (!running_ && job_q_.empty()) break;
        if (job_q_.empty()) continue;
        frame = std::move(job_q_.front().frame);
        job_q_.pop_front();
      }

      // BGR→RGB（模型需要 RGB）
      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

      image_buffer_t src;
      std::memset(&src, 0, sizeof(src));
      src.height = frame.rows;
      src.width = frame.cols;
      src.width_stride = frame.step[0];
      src.virt_addr = frame.data;
      src.format = IMAGE_FORMAT_RGB888;
      src.size = frame.total() * frame.elemSize();

      letterbox_t letter_box;
      std::memset(&letter_box, 0, sizeof(letter_box));
      if (convert_image_with_letterbox(&src, &dst_img, &letter_box, 114) < 0) continue;

      rknn_input inputs[1];
      std::memset(inputs, 0, sizeof(inputs));
      inputs[0].index = 0;
      inputs[0].type = RKNN_TENSOR_UINT8;
      inputs[0].fmt = RKNN_TENSOR_NHWC;
      inputs[0].size = ctx_[wid].model_width * ctx_[wid].model_height * ctx_[wid].model_channel;
      inputs[0].buf = dst_img.virt_addr;
      rknn_inputs_set(ctx_[wid].rknn_ctx, 1, inputs);

      auto t0 = std::chrono::steady_clock::now();
      rknn_run(ctx_[wid].rknn_ctx, nullptr);

      rknn_output outputs[MAX_OUT_TENSORS];
      std::memset(outputs, 0, sizeof(outputs));
      for (uint32_t i = 0; i < ctx_[wid].io_num.n_output; i++) {
        outputs[i].index = i;
        outputs[i].want_float = !ctx_[wid].is_quant;
      }
      rknn_outputs_get(ctx_[wid].rknn_ctx, ctx_[wid].io_num.n_output, outputs, nullptr);
      double npu_ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
      {
        std::lock_guard<std::mutex> lk(stat_mtx_);
        npu_acc_ += npu_ms;
        npu_cnt_++;
      }

      object_detect_result_list od;
      std::memset(&od, 0, sizeof(od));
      post_process(&ctx_[wid], outputs, &letter_box,
                   static_cast<float>(conf_th_), static_cast<float>(nms_th_), &od);
      rknn_outputs_release(ctx_[wid].rknn_ctx, ctx_[wid].io_num.n_output, outputs);

      publish_detections(od);
      hud_publish(od, frame);
      processed_++;
    }
  }

  // ── HUD：画框 + JPEG 编码（hud_fps 节流）→ /image_det（CompressedImage）──
  // 手机端 robot_ui 的 /cam.mjpg 直接搬运本话题字节，不重复解码/渲染
  // 用 OpenCV 绘制（自带边界裁剪、线程安全），不用 demo 的 draw_*
  void hud_publish(const object_detect_result_list &od, const cv::Mat &frame_rgb) {
    if (!hud_enable_ || frame_rgb.empty()) return;
    auto t_now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lk(hud_mtx_);
      if (t_now - last_hud_ < hud_interval_) return;
      last_hud_ = t_now;
    }
    cv::Mat bgr;
    cv::cvtColor(frame_rgb, bgr, cv::COLOR_RGB2BGR);
    for (int i = 0; i < od.count; i++) {
      const auto &d = od.results[i];
      cv::rectangle(bgr, cv::Point(d.box.left, d.box.top),
                    cv::Point(d.box.right, d.box.bottom), cv::Scalar(255, 0, 0), 2);
      char text[64];
      snprintf(text, sizeof(text), "%s %.0f%%",
               coco_cls_to_name(d.cls_id), d.prop * 100);
      cv::putText(bgr, text, cv::Point(d.box.left, std::max(14, d.box.top - 6)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    std::vector<unsigned char> buf;
    if (!cv::imencode(".jpg", bgr, buf, {cv::IMWRITE_JPEG_QUALITY, hud_quality_})) return;
    sensor_msgs::msg::CompressedImage m;
    m.header.stamp = now();
    m.header.frame_id = "camera";
    m.format = "jpeg";
    m.data.assign(buf.begin(), buf.end());
    pub_hud_->publish(m);
  }

  void publish_detections(const object_detect_result_list &od) {
    std_msgs::msg::Float32MultiArray m;
    m.layout.dim.resize(2);
    m.layout.dim[0].label = "boxes";
    m.layout.dim[0].size = od.count;
    m.layout.dim[0].stride = od.count * 6;
    m.layout.dim[1].label = "cls,conf,x1,y1,x2,y2";
    m.layout.dim[1].size = 6;
    m.layout.dim[1].stride = 6;
    m.layout.data_offset = 0;
    m.data.reserve(od.count * 6);
    for (int i = 0; i < od.count; i++) {
      const auto &d = od.results[i];
      m.data.push_back(static_cast<float>(d.cls_id));
      m.data.push_back(d.prop);
      m.data.push_back(static_cast<float>(d.box.left));
      m.data.push_back(static_cast<float>(d.box.top));
      m.data.push_back(static_cast<float>(d.box.right));
      m.data.push_back(static_cast<float>(d.box.bottom));
    }
    pub_->publish(m);

    // 检测概览日志（1s 节流，验收用）
    if (od.count > 0) {
      std::string s;
      for (int i = 0; i < od.count && i < 5; i++) {
        s += coco_cls_to_name(od.results[i].cls_id);
        s += " " + std::to_string(static_cast<int>(od.results[i].prop * 100)) + "% ";
      }
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "检测到 %d 目标: %s", od.count, s.c_str());
    }
  }

  void print_stats() {
    int processed = processed_.exchange(0);
    int cnt;
    double acc;
    {
      std::lock_guard<std::mutex> lk(stat_mtx_);
      cnt = npu_cnt_.exchange(0);
      acc = npu_acc_;
      npu_acc_ = 0.0;
    }
    double fps = processed / 5.0;
    double npu = cnt > 0 ? acc / cnt : 0.0;
    RCLCPP_INFO(get_logger(), "端到端 %.1f FPS | NPU 均耗 %.1f ms | 处理 %d 帧",
                fps, npu, processed);
  }

  // 参数
  std::string model_path_;
  double conf_th_{0.25}, nms_th_{0.45};
  int max_jobs_{8};
  bool hud_enable_{true};
  double hud_fps_{10.0};
  int hud_quality_{70};

  // HUD 节流
  std::mutex hud_mtx_;
  std::chrono::steady_clock::time_point last_hud_{};
  std::chrono::milliseconds hud_interval_{100};

  // NPU
  rknn_app_context_t ctx_[N_CORES];
  unsigned char *dst_buf_[N_CORES] = {nullptr};

  // 任务队列
  std::deque<Job> job_q_;
  std::mutex job_mtx_;
  std::condition_variable cv_;

  // 统计
  std::atomic<bool> running_{true};
  std::atomic<int> processed_{0};
  std::atomic<int> npu_cnt_{0};
  double npu_acc_{0.0};
  std::mutex stat_mtx_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_hud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::vector<std::thread> workers_;
};

int main(int argc, char **argv) {
  // postprocess.cc 以相对路径 ./model/coco_80_labels_list.txt 读标签，
  // chdir 到包 share 目录后，model_path 默认值 "model/yolo11.rknn" 也相对该目录
  if (chdir(RKNN_SHARE_DIR) != 0) {
    fprintf(stderr, "chdir(%s) 失败，标签文件可能找不到\n", RKNN_SHARE_DIR);
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RknnInfer>());
  rclcpp::shutdown();
  return 0;
}
