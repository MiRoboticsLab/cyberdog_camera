// Copyright (c) 2022 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef CAMERA_API__CAMERA_API_HPP_
#define CAMERA_API__CAMERA_API_HPP_

#include <stdint.h>
#include <opencv2/opencv.hpp>

namespace cyberdog
{
namespace camera
{

typedef void * CameraHandle;
typedef int (* FrameCallback)(cv::Mat & frame, uint64_t timestamp, void * args);

enum ImageFormat
{
  kImageFormatBGR = 0,
  kImageFormatRGB,
  kImageFormatInvalid,
};

CameraHandle OpenCamera(int camera_id, int & status);
int CloseCamera(CameraHandle);
int StartStream(CameraHandle handle, ImageFormat format,
  int width, int height, FrameCallback cb, void * cb_args);
int StopStream(CameraHandle handle);

}  // namespace camera
}  // namespace cyberdog

#endif  // CAMERA_API__CAMERA_API_HPP_
