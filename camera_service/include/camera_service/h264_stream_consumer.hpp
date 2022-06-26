// Copyright (c) 2021  Xiaomi Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CAMERA_SERVICE__H264_STREAM_CONSUMER_HPP_
#define CAMERA_SERVICE__H264_STREAM_CONSUMER_HPP_

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nvosd.h>
#include <string>
#include <memory>
#include <vector>
#include "camera_base/stream_consumer.hpp"
#include "camera_base/video_encoder.hpp"
#include "ros2_service.hpp"

namespace cyberdog
{
namespace camera
{

using StreamCb = std::function<void(uint8_t *, int64_t, uint16_t)>;

class H264StreamConsumer : public StreamConsumer
{
public:
  explicit H264StreamConsumer(Size2D<uint32_t> size, StreamCb callback = nullptr);
  virtual ~H264StreamConsumer();

  virtual bool threadInitialize();
  virtual bool threadShutdown();
  virtual bool processBuffer(Buffer * buffer);

private:
  static void inputDoneCallback(int dmabuf_fd, void * arg)
  {
    H264StreamConsumer * thiz = static_cast<H264StreamConsumer *>(arg);
    thiz->inputDoneCallback(dmabuf_fd);
  }
  static void outputDoneCallback(uint8_t * data, size_t size, int64_t ts, void * arg)
  {
    H264StreamConsumer * thiz = static_cast<H264StreamConsumer *>(arg);
    thiz->outputDoneCallback(data, size, ts);
  }

  void inputDoneCallback(int fd);
  void outputDoneCallback(uint8_t * data, size_t size, int64_t ts);
  void publishImage(uint64_t frame_id, ImageBuffer & buf);
  void publishH264Image(uint8_t * data, size_t size, int64_t timestamp);
  void bodySubCallback(const BodyInfoT::SharedPtr msg);
  void drawBodyRectangles(int fd);

  bool startSoundTimer();
  void stopSoundTimer();
  static void playVideoSound(union sigval val);

  VideoEncoder * m_videoEncoder;
  void * m_nvosdContext;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_h264Publisher;
  timer_t m_timer;

  rclcpp::Subscription<BodyInfoT>::SharedPtr m_bodySub;
  std::vector<BodyT> m_currentBodyRects;
  std::mutex m_rectLock;

  // for live stream
  StreamCb live_stream_cb_;
};

}  // namespace camera
}  // namespace cyberdog

#endif  // CAMERA_SERVICE__H264_STREAM_CONSUMER_HPP_
