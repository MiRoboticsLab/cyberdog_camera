#define LOG_TAG "STEREO_CAMERA"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "camera_api/camera_api.hpp"
#include "camera_utils/camera_log.hpp"
#include "camera_utils/global_timestamp_reader.h"
#include "cyberdog_visions_interfaces/msg/metadata.hpp"

#define ENABLE_CAM_INFINITE 0

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
    CameraTopic(rclcpp_lifecycle::LifecycleNode * parent, int camera_id, const std::string & name);
    ~CameraTopic();

    bool Initialize(int width, int height, ImageFormat format);
    bool Destroy();

  private:
    static int sFrameCallback(cv::Mat & frame, uint64_t timestamp,uint32_t capture_id, void * arg);
    void PublishImage(cv::Mat & frame, uint64_t timestamp,uint32_t frame_number);
    void publishMetadata(uint64_t t_real_sys, uint64_t t_hw,uint32_t frame_number);

    std::string name_;
    rclcpp_lifecycle::LifecycleNode *parent_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp_lifecycle::LifecyclePublisher<cyberdog_visions_interfaces::msg::Metadata>::SharedPtr pub_metadata;

    CameraHandle cam_hdl_;
    int camera_id_;
    int width_;
    int height_;
    ImageFormat format_;
};

class StereoCameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  StereoCameraNode(const std::string & name);
  ~StereoCameraNode();

protected:
  //  override LifecycleNode methods
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    CAM_INFO("configure");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    CAM_INFO("activate");
    Initialize();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    CAM_INFO("deactivate");
    Shutdown();
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    CAM_INFO("cleanup");
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    CAM_INFO("shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  bool Initialize();
  bool Shutdown();

  void InitParameters();
  std::vector<CameraTopic *> topic_list_;
};

CameraTopic::CameraTopic(rclcpp_lifecycle::LifecycleNode * parent,
        int camera_id, const std::string & name)
: name_(name),
  parent_(parent),
  cam_hdl_(nullptr),
  camera_id_(camera_id)
{
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  pub_ = parent_->create_publisher<sensor_msgs::msg::Image>("image_" + name, qos);
  pub_->on_activate();
  if(name_ == "left")
  {
    pub_metadata = parent_->create_publisher<cyberdog_visions_interfaces::msg::Metadata>("image_" + name_+"_metadata", qos);
    pub_metadata->on_activate();
  }
}

CameraTopic::~CameraTopic()
{
}

bool CameraTopic::Initialize(int width, int height, ImageFormat format)
{
  int status;

  format_ = format;
  cam_hdl_ = OpenCamera(camera_id_, status, true);
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

int CameraTopic::sFrameCallback(cv::Mat & frame, uint64_t timestamp, uint32_t capture_id,void * arg)
{
  CameraTopic *_this = reinterpret_cast<CameraTopic *>(arg);

  _this->PublishImage(frame, timestamp,capture_id);
  
  return 0;
}

rclcpp::Clock clock;
#define LOG_MILLSECONDS(ms, fmt, ...) do { \
  RCLCPP_INFO_THROTTLE( \
    rclcpp::get_logger(LOG_TAG), clock, ms, "%s: " fmt, __FUNCTION__, ##__VA_ARGS__); \
} while (0)

void CameraTopic::PublishImage(cv::Mat & frame, uint64_t timestamp,uint32_t frame_number)
{
  auto msg = std::make_unique<sensor_msgs::msg::Image>();

  double hw_time = timestamp*0.000001;
  double real_ts_sys = get_frame_timestamp(hw_time)*0.001;
  uint64_t real_ts_sys_nano = real_ts_sys*1000000000;

  switch (format_) {
    case kImageFormatBGR:
      msg->encoding = "bgr8";
      break;
    case kImageFormatRGB:
      msg->encoding = "rgb8";
      break;
    case kImageFormatGRAY:
      msg->encoding = "mono8";
      break;
    default:
      break;
  }
  msg->is_bigendian = false;
  msg->width = frame.cols;
  msg->height = frame.rows;
  msg->step = frame.step;
  size_t size = msg->step * msg->height;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = name_ + "_link";
  msg->header.stamp = rclcpp::Time(real_ts_sys_nano);
  pub_->publish(std::move(msg));
  if(name_ == "left")
  {
    publishMetadata(real_ts_sys_nano,timestamp,frame_number);
  }

  LOG_MILLSECONDS(3000, "%s Publishing image #%zu", name_.c_str(), frame_number);
}

void CameraTopic::publishMetadata(uint64_t t_real_sys, uint64_t t_hw, uint32_t frame_number)
{  
  cyberdog_visions_interfaces::msg::Metadata msg;
  msg.header.frame_id = name_ + "_link";
  msg.header.stamp = rclcpp::Time(t_real_sys);
  std::stringstream json_data;
  const char* separator = ",";
  json_data << "{";
  json_data << "\"" << "frame_number" << "\":" << frame_number;
  json_data << separator << "\"" << "frame_timestamp" << "\":" << std::fixed << t_real_sys;
  json_data << separator << "\"" << "frame_hw" << "\":" << std::fixed << t_hw;
  json_data << "}";
  msg.json_data = json_data.str();
  pub_metadata->publish(msg);
}

StereoCameraNode::StereoCameraNode(const std::string & name)
  : rclcpp_lifecycle::LifecycleNode(name)
{
  InitParameters();
}

StereoCameraNode::~StereoCameraNode()
{
  Shutdown();
}

struct topic_info {
  int camera_id;
  int width;
  int height;
  ImageFormat format;
  std::string name;
};

topic_info g_all_topics[] =
{
  {1, 640, 480, kImageFormatBGR, "rgb"},
  {2, 640, 400, kImageFormatBGR, "right"},
  {3, 640, 400, kImageFormatBGR, "left"}
};

topic_info g_stereo_topics[] =
{
  {2, 640, 400, kImageFormatBGR, "right"},
  {3, 640, 400, kImageFormatBGR, "left"}
};

bool StereoCameraNode::Initialize()
{
  bool stereo_only;
  globalTime.enable_time_diff_keeper(true);


  topic_info *topics;
  size_t count = 0;
  this->get_parameter_or("stereo_only", stereo_only, false);
  if (stereo_only) {
    topics = g_stereo_topics;
    count = sizeof(g_stereo_topics) / sizeof(g_stereo_topics[0]);
  } else {
    topics = g_all_topics;
    count = sizeof(g_all_topics) / sizeof(g_all_topics[0]);
  }

  for (size_t i = 0; i < count; i++) {
    int w, h, format;
    CameraTopic * topic = new CameraTopic(this,
        topics[i].camera_id, topics[i].name);

    this->get_parameter("w_" + topics[i].name, w);
    this->get_parameter("h_" + topics[i].name, h);
    this->get_parameter("format_" + topics[i].name, format);
    if (topic->Initialize(w, h, (ImageFormat)format)) {
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
  topic_list_.clear();

  return true;
}

void StereoCameraNode::InitParameters()
{
  this->declare_parameter("stereo_only", false);
  for (size_t i = 0; i < sizeof(g_all_topics) / sizeof(g_all_topics[0]); i++) {
    this->declare_parameter("w_" + g_all_topics[i].name, g_all_topics[i].width);
    this->declare_parameter("h_" + g_all_topics[i].name, g_all_topics[i].height);
    this->declare_parameter("format_" + g_all_topics[i].name, (int)g_all_topics[i].format);
  }
}

}  // namespace camera
}  // namespace cyberdog

int main(int argc, char** argv)
{
#if ENABLE_CAM_INFINITE
  setenv("enableCamInfiniteTimeout", "1", 1);
#endif
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto stereo_node = std::make_shared<cyberdog::camera::StereoCameraNode>("stereo_camera");
  exec.add_node(stereo_node->get_node_base_interface());

  exec.spin();
  rclcpp::shutdown();
}
