
/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rknn_api.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <string>


using namespace std;
using namespace cv;

/*-------------------------------------------
                  Functions
-------------------------------------------*/

static void dump_tensor_attr(rknn_tensor_attr* attr)
{
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

static unsigned char* load_model(const char* filename, int* model_size)
{
  FILE* fp = fopen(filename, "rb");
  if (fp == nullptr) {
    printf("fopen %s fail!\n", filename);
    return NULL;
  }
  fseek(fp, 0, SEEK_END);
  int            model_len = ftell(fp);
  unsigned char* model     = (unsigned char*)malloc(model_len);
  fseek(fp, 0, SEEK_SET);
  if (model_len != fread(model, 1, model_len, fp)) {
    printf("fread %s fail!\n", filename);
    free(model);
    return NULL;
  }
  *model_size = model_len;
  if (fp) {
    fclose(fp);
  }
  return model;
}

static int rknn_GetTop(float* pfProb, float* pfMaxProb, uint32_t* pMaxClass, uint32_t outputCount, uint32_t topNum)
{
  uint32_t i, j;

#define MAX_TOP_NUM 20
  if (topNum > MAX_TOP_NUM)
    return 0;

  memset(pfMaxProb, 0, sizeof(float) * topNum);
  memset(pMaxClass, 0xff, sizeof(float) * topNum);

  for (j = 0; j < topNum; j++) {
    for (i = 0; i < outputCount; i++) {
      if ((i == *(pMaxClass + 0)) || (i == *(pMaxClass + 1)) || (i == *(pMaxClass + 2)) || (i == *(pMaxClass + 3)) ||
          (i == *(pMaxClass + 4))) {
        continue;
      }

      if (pfProb[i] > *(pfMaxProb + j)) {
        *(pfMaxProb + j) = pfProb[i];
        *(pMaxClass + j) = i;
      }
    }
  }

  return 1;
}


/*-------------------------------------------
                  ImageNet Labels
-------------------------------------------*/
static std::vector<std::string> g_labels;

static void load_labels(const char* path) {
  std::ifstream f(path);
  if (!f.is_open()) return;
  std::string line;
  while (std::getline(f, line)) {
    if (!line.empty()) g_labels.push_back(line);
  }
}

static std::string label_of(int idx) {
  if (g_labels.empty()) return std::to_string(idx);
  if (idx < 0 || idx >= (int)g_labels.size()) return std::to_string(idx);
  return g_labels[idx];
}

int main(int argc, char** argv) {

  const int MODEL_IN_WIDTH    = 224;
  const int MODEL_IN_HEIGHT   = 224;

  rknn_context ctx = 0;
  int            ret;
  int            model_len = 0;
  unsigned char* model;

  const char* model_path = argv[1];

  // 加载 ImageNet 类别名称（文件缺失时退回显示数字）
  load_labels("imagenet_labels.txt");
  if (g_labels.empty()) load_labels("model/synset_words.txt");
  if (g_labels.empty()) load_labels("synset_words.txt");
  if (!g_labels.empty())
    printf("已加载 %zu 条 ImageNet 类别名称\n", g_labels.size());

    // 优先用 V4L2 后端直接打开 USB 摄像头（比 GStreamer 在嵌入式上更稳）
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        cap.open(0);  // 退而求其次尝试默认后端
    }
    if (!cap.isOpened()) {
        std::cout << "无法打开摄像头 (/dev/video0)" << std::endl;
        return -1;
    }

    cv::Mat frame;
    // 仅在存在 X 显示时创建窗口，否则纯命令行运行（无显示器也能跑）
    bool has_display = (getenv("DISPLAY") != nullptr);
    if (has_display) {
        cv::namedWindow("Video", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Video", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    }
    
    cv::Mat orig_img_rgb;
    
    // Load RKNN Model
        model = load_model(model_path, &model_len);
        ret   = rknn_init(&ctx, model, model_len, 0, NULL);
        if (ret < 0) {
    		printf("rknn_init fail! ret=%d\n", ret);
    		return -1;
  		}
    while (true) {
        cap.read(frame);
        cv::rotate(frame,frame,cv::ROTATE_90_COUNTERCLOCKWISE);
        if (frame.empty()) {
            std::cout << "无法获取帧" << std::endl;
            break;
        }
        
        
        cv::cvtColor(frame, orig_img_rgb, cv::COLOR_BGR2RGB);
        
        cv::Mat img = orig_img_rgb.clone();
        if (frame.cols != MODEL_IN_WIDTH || frame.rows != MODEL_IN_HEIGHT) {
    		printf("resize %d %d to %d %d\n", frame.cols, frame.rows,MODEL_IN_WIDTH,MODEL_IN_HEIGHT);
    		cv::resize(frame, img, cv::Size(MODEL_IN_WIDTH, MODEL_IN_HEIGHT), 0, 0, cv::INTER_LINEAR);
  		}
  		
    	// Get Model Input Output Info
  		rknn_input_output_num io_num;
  		ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  		if (ret != RKNN_SUCC) {
    		printf("rknn_query fail! ret=%d\n", ret);
    		return -1;
  		}
  		printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

  		printf("input tensors:\n");
  		rknn_tensor_attr input_attrs[io_num.n_input];
  		memset(input_attrs, 0, sizeof(input_attrs));
  		for (int i = 0; i < io_num.n_input; i++) {
    		input_attrs[i].index = i;
    		ret  = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    		if (ret != RKNN_SUCC) {
      			printf("rknn_query fail! ret=%d\n", ret);
      			return -1;
    		}
    		dump_tensor_attr(&(input_attrs[i]));
  		}

  		printf("output tensors:\n");
  		rknn_tensor_attr output_attrs[io_num.n_output];
  		memset(output_attrs, 0, sizeof(output_attrs));
  		for (int i = 0; i < io_num.n_output; i++) {
    		output_attrs[i].index = i;
    		ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    		if (ret != RKNN_SUCC) {
      			printf("rknn_query fail! ret=%d\n", ret);
      			return -1;
    		}
    		dump_tensor_attr(&(output_attrs[i]));
  		}

  	// Set Input Data
  	rknn_input inputs[1];
  	memset(inputs, 0, sizeof(inputs));
  	inputs[0].index = 0;
  	inputs[0].type  = RKNN_TENSOR_UINT8;
  	inputs[0].size  = img.cols * img.rows * img.channels() * sizeof(uint8_t);
  	inputs[0].fmt   = RKNN_TENSOR_NHWC;
  	inputs[0].buf   = img.data;

  	ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
  	if (ret < 0) {
    	printf("rknn_input_set fail! ret=%d\n", ret);
    	return -1;
  	}

  	// Run
  	printf("rknn_run\n");
  	ret = rknn_run(ctx, nullptr);
  	if (ret < 0) {
    	printf("rknn_run fail! ret=%d\n", ret);
    	return -1;
  	}

  	// Get Output
  	rknn_output outputs[1];
  	memset(outputs, 0, sizeof(outputs));
  	outputs[0].want_float = 1;
  	ret = rknn_outputs_get(ctx, 1, outputs, NULL);
  	if (ret < 0) {
    	printf("rknn_outputs_get fail! ret=%d\n", ret);
    	return -1;
  	}
    	//cv::rotate(frame,dst,cv::ROTATE_90_COUNTERCLOCKWISE);

  	// Post Process
  	for (int i = 0; i < io_num.n_output; i++) {
    	uint32_t MaxClass[5];
    	float    fMaxProb[5];
    	float*   buffer = (float*)outputs[i].buf;
    	uint32_t sz     = outputs[i].size / 4;

    	rknn_GetTop(buffer, fMaxProb, MaxClass, sz, 5);

    	printf(" --- Top5 ---\n");
    	for (int i = 0; i < 5; i++) {
      		printf("  %3d: %8.6f  %s\n", MaxClass[i], fMaxProb[i], label_of(MaxClass[i]).c_str());
    	}

    	// 仅在画面上绘制置信度最高的（Top1）标签与概率
    	{
      	std::string name = label_of(MaxClass[0]);
      	if (name.size() > 28) name = name.substr(0, 28);
      	putText(frame, name, Point(32, 64), FONT_HERSHEY_TRIPLEX, 1.2, Scalar(0,0,255),2,8,0);

      	std::string prob = std::to_string(fMaxProb[0]);
      	putText(frame, prob, Point(440, 64), FONT_HERSHEY_TRIPLEX, 1.2, Scalar(0,0,255),2,8,0);
    	}
  	}
  	// Release rknn_outputs
    rknn_outputs_release(ctx, 1, outputs);
    
    

    if (has_display) {
        cv::imshow("Video", frame);
        if (cv::waitKey(1) == 'q') {
            rknn_outputs_release(ctx, 1, outputs);
            break;
        }
    } else {
        // 无显示：每 15 帧打印一次 Top5，按 Ctrl+C 退出
        static int fcnt = 0;
        if (++fcnt % 15 == 0) {
            printf("  [frame %d] see Top5 above\n", fcnt);
        }
    }
}

    cap.release();
    cv::destroyAllWindows();
    
    // Release
    if (ctx > 0)
    {
      rknn_destroy(ctx);
    }
    if (model) {
      free(model);
    }

    return 0;
}

