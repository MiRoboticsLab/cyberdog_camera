#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "camera_api/camera_api.hpp"
#include "global_timestamp_reader.h"

namespace cyberdog
{
namespace camera
{

global_time_interface globalTime;

double get_frame_timestamp(double frame_time)
{
  auto sp = globalTime._tf_keeper;
  bool ts_is_ready;
  if (sp)
      frame_time = sp->get_system_hw_time(frame_time, ts_is_ready);
  else
      printf("Notification: global_timestamp_reader - time_diff_keeper is being shut-down");

  return frame_time;
}

class CameraTopic
{
  public:
    CameraTopic(rclcpp::Node * parent, int camera_id, const std::string & name);
    ~CameraTopic();

    bool Initialize(int width, int height, ImageFormat format);
    bool Destroy();

  private: 
    static int sFrameCallback(cv::Mat & frame, uint64_t timestamp, void * arg);
    void PublishImage(cv::Mat & frame, uint64_t timestamp);

    std::string name_;
    rclcpp::Node *parent_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

    CameraHandle cam_hdl_;
    int camera_id_;
    int width_;
    int height_;
    ImageFormat format_;
};

class StereoCameraNode : public rclcpp::Node
{
public:
  StereoCameraNode(const std::string & name);
  ~StereoCameraNode();

  bool Initialize();
  bool Shutdown();

private:
  std::vector<CameraTopic *> topic_list_;
};

CameraTopic::CameraTopic(rclcpp::Node * parent, int camera_id, const std::string & name)
: name_(name),
  parent_(parent),
  cam_hdl_(nullptr),
  camera_id_(camera_id)
{
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  pub_ = parent_->create_publisher<sensor_msgs::msg::Image>("image_" + name, qos);
}

CameraTopic::~CameraTopic()
{
}

bool CameraTopic::Initialize(int width, int height, ImageFormat format)
{
  int status;

  cam_hdl_ = OpenCamera(camera_id_, status);
  if (!cam_hdl_) {
    return false;
  }

  if (0 != StartStream(cam_hdl_, format, width, height, sFrameCallback, this)) {
    CloseCamera(cam_hdl_);
    return false;
  }

  return true;
}

bool CameraTopic::Destroy()
{
  StopStream(cam_hdl_);
  CloseCamera(cam_hdl_);

  return true;
}

int CameraTopic::sFrameCallback(cv::Mat & frame, uint64_t timestamp, void * arg)
{
  CameraTopic *_this = reinterpret_cast<CameraTopic *>(arg);

  _this->PublishImage(frame, timestamp);

  return 0;
}

void CameraTopic::PublishImage(cv::Mat & frame, uint64_t timestamp)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();

  double hw_time = timestamp*0.000001;
  double real_ts_sys = get_frame_timestamp(hw_time)*0.001;
  uint32_t sec = (uint32_t)real_ts_sys;
  int32_t nsec = (real_ts_sys - sec) * 1000 * 1000 * 1000;

  msg->is_bigendian = false;
  msg->width = frame.cols;
  msg->height = frame.rows;
  msg->encoding = "bgr8";
  msg->step = frame.step;
  size_t size = msg->step * msg->height;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = name_ + "_link";
  msg->header.stamp.sec = sec;
  msg->header.stamp.nanosec = nsec;

  pub_->publish(std::move(msg));

}

StereoCameraNode::StereoCameraNode(const std::string & name)
  : rclcpp::Node(name)
{
  Initialize();
}

StereoCameraNode::~StereoCameraNode()
{
  Shutdown();
}

struct {
  int camera_id;
  int width;
  int height;
  ImageFormat format;
  std::string name;
} g_topics[3] = 
{
  {1, 640, 480, kImageFormatBGR, "rgb"},
  {2, 640, 400, kImageFormatBGR, "left"},
  {3, 640, 400, kImageFormatBGR, "right"}
};

bool StereoCameraNode::Initialize()
{
  globalTime.enable_time_diff_keeper(true);

  for (int i = 0; i < 3; i++) {
    int w, h;
    CameraTopic * topic = new CameraTopic(this,
        g_topics[i].camera_id, g_topics[i].name);

    this->declare_parameter("w_" + g_topics[i].name, g_topics[i].width);
    this->declare_parameter("h_" + g_topics[i].name, g_topics[i].height);
    this->get_parameter("w_" + g_topics[i].name, w);
    this->get_parameter("h_" + g_topics[i].name, h);
    if (topic->Initialize(w, h, g_topics[i].format)) {
      topic_list_.push_back(topic);
    } else {
      delete topic;
    }
  }

  return true;
}

bool StereoCameraNode::Shutdown()
{
  for (size_t i = 0; i < topic_list_.size(); i++) {
    topic_list_[i]->Destroy();
    delete topic_list_[i];
  }
  return true;
}

}  // namespace camera
}  // namespace cyberdog

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto my_node = std::make_shared<cyberdog::camera::StereoCameraNode>("stereo_camera");
  exec.add_node(my_node);

  exec.spin();
  rclcpp::shutdown();
}
