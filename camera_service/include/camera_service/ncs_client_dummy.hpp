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

#ifndef CAMERA_SERVICE__NCS_CLIENT_DUMMY_HPP_
#define CAMERA_SERVICE__NCS_CLIENT_DUMMY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include "./ros2_service.hpp"

namespace cyberdog
{
namespace camera
{

enum SoundType
{
  SoundShutter = 0,
  SoundRecordStart,
  SoundRecording,
  SoundLiveStart,
  SoundFaceAddStart,
  SoundFaceNumWarn,
  SoundFacePoseWarn,
  SoundFaceDistWarn,
  SoundFaceAddEnd,
  SoundFaceAddFailed,
  SoundFaceDetect,
  SoundTypeNone,
};

class NCSClient
{
public:
  static NCSClient & getInstance();
  bool play(SoundType type);
  bool requestLed(bool on);

private:
  NCSClient();
  ~NCSClient();
};

}  // namespace camera
}  // namespace cyberdog

#endif  // CAMERA_SERVICE__NCS_CLIENT_DUMMY_HPP_
