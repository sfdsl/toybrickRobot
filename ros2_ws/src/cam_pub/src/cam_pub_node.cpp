// cam_pub_node.cpp
// USB 相机采集节点：V4L2 采集 -> /image_raw
// 设计要点（《13_视觉识别实验方案》P1）：
//   - MJPG 640x480@30（v4l2-ctl 实测支持），CPU 解码、USB 带宽小
//   - CAP_V4L2 显式后端 + BUFFERSIZE=2（实测：=1 会触发 OpenCV V4L2 后端
//     流水线断档、帧率砍半 15fps；=2 满速 30fps，防旧帧由 QoS depth=1 负责）
//   - QoS depth=1：推理慢一拍时丢旧帧，视觉要"现在"不要"历史"
//   - 采集放独立线程，读失败自动重开
#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CamPub : public rclcpp::Node
{
public:
  CamPub() : Node("cam_pub")
  {
    index_   = declare_parameter<int>("camera_index", 0);
    width_   = declare_parameter<int>("width", 640);
    height_  = declare_parameter<int>("height", 480);
    fps_     = declare_parameter<int>("fps", 30);
    fourcc_  = declare_parameter<std::string>("fourcc", "MJPG");
    topic_   = declare_parameter<std::string>("topic", "image_raw");

    pub_ = create_publisher<sensor_msgs::msg::Image>(topic_, 1);

    open_camera();

    worker_ = std::thread([this] { loop(); });
    RCLCPP_INFO(get_logger(),
                "cam_pub 启动: /dev/video%d %s %dx%d@%d -> /%s",
                index_, fourcc_.c_str(), width_, height_, fps_, topic_.c_str());
  }

  ~CamPub() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

private:
  bool open_camera()
  {
    cap_.open(index_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "打开 /dev/video%d 失败，2s 后重试", index_);
      return false;
    }
    int code = static_cast<int>(fourcc_[0]) | (static_cast<int>(fourcc_[1]) << 8) |
               (static_cast<int>(fourcc_[2]) << 16) | (static_cast<int>(fourcc_[3]) << 24);
    cap_.set(cv::CAP_PROP_FOURCC, code);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 2);   // 实测 =1 时帧率砍半，见文件头注释
    double w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    double h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    RCLCPP_INFO(get_logger(), "相机已打开，实际分辨率 %.0fx%.0f", w, h);
    return true;
  }

  void loop()
  {
    using clock = std::chrono::steady_clock;
    auto t_last = clock::now();
    int frames = 0;

    while (rclcpp::ok() && running_) {
      if (!cap_.isOpened()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        open_camera();
        continue;
      }
      cv::Mat frame;
      if (!cap_.read(frame) || frame.empty()) {
        RCLCPP_WARN(get_logger(), "读帧失败，尝试重开相机");
        cap_.release();
        continue;
      }
      if (frame.cols != width_ || frame.rows != height_)
        cv::resize(frame, frame, cv::Size(width_, height_));

      sensor_msgs::msg::Image::SharedPtr msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      msg->header.stamp = now();
      msg->header.frame_id = "camera";
      pub_->publish(*msg);

      ++frames;
      auto now = clock::now();
      if (now - t_last >= std::chrono::seconds(5)) {
        double fps = frames / std::chrono::duration<double>(now - t_last).count();
        RCLCPP_INFO(get_logger(), "发布帧率: %.1f FPS", fps);
        frames = 0;
        t_last = now;
      }
    }
    cap_.release();
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  cv::VideoCapture cap_;
  std::thread worker_;
  std::atomic<bool> running_{true};
  int index_{0}, width_{640}, height_{480}, fps_{30};
  std::string fourcc_{"MJPG"}, topic_{"image_raw"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CamPub>());
  rclcpp::shutdown();
  return 0;
}
