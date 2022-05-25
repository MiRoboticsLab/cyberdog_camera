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

#define LOG_TAG "NCSClient"
#include <memory>
#include "camera_service/ncs_client_dummy.hpp"
#include "camera_service/camera_manager.hpp"
#include "camera_utils/camera_log.hpp"

namespace cyberdog
{
namespace camera
{

static const char * AUDIO_SERVER_NAME = "audio_play";
static const char * LED_SERVER_NAME = "athena_led";

static int gSoundAudioArrays[SoundTypeNone] =
{
  [SoundShutter] = 8,
  [SoundRecordStart] = 122,
  [SoundRecording] = 9,
  [SoundLiveStart] = 118,
  [SoundFaceAddStart] = 103,
  [SoundFaceNumWarn] = 113,
  [SoundFacePoseWarn] = 115,
  [SoundFaceDistWarn] = 114,
  [SoundFaceAddEnd] = 109,
  [SoundFaceAddFailed] = 108,
  [SoundFaceDetect] = 5,
};

NCSClient & NCSClient::getInstance()
{
  static NCSClient s_instance;

  return s_instance;
}

NCSClient::NCSClient()
{
  CAM_INFO("Create");
}

NCSClient::~NCSClient()
{
  CAM_INFO("Destroy");
}

bool NCSClient::play(SoundType type)
{
  bool ret = true;

  return ret;
}

bool NCSClient::requestLed(bool on)
{
  bool ret = true;

  return ret;
}


}  // namespace camera
}  // namespace cyberdog
