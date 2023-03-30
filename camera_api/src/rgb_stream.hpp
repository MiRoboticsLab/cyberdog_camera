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

#ifndef RGB_STREAM_HPP_
#define RGB_STREAM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include "camera_base/stream_consumer.hpp"
#include "camera_api/camera_api.hpp"
#include "camera_base/color_convert.hpp"

namespace cyberdog
{
namespace camera
{

class RgbStream : public StreamConsumer
{
public:
  RgbStream(Size2D<uint32_t> size, ImageFormat format, FrameCallback cb, void * cb_args);
  virtual ~RgbStream();

  virtual bool threadInitialize();
  virtual bool threadShutdown();
  virtual bool processBuffer(Buffer * buffer);

  void setName(const std::string &name)
  {
    name_ = name;
  }

private:
  void publishImage(ImageBuffer & buf);

  ImageFormat format_;
  FrameCallback callback_;
  void *cb_args_;
  int rgba_fd_;
  ColorConvert * format_convert_;
  std::string name_;
};

}  // namespace camera
}  // namespace cyberdog

#endif  // RGB_STREAM_HPP_
