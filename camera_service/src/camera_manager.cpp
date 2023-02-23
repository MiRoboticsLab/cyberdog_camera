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

#define LOG_TAG "CameraManager"
#include <stdio.h>
#include <sys/stat.h>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include "camera_service/camera_manager.hpp"
#include "camera_utils/camera_utils.hpp"
#include "camera_utils/camera_log.hpp"

#define CAMERA_PATH "/home/mi/Camera/"

namespace cyberdog
{
namespace camera
{

CameraManager * CameraManager::getInstance()
{
  static CameraManager s_instance;

  return &s_instance;
}

CameraManager::CameraManager()
: m_camera(NULL)
{
}

CameraManager::~CameraManager()
{
}

bool CameraManager::openCamera(int camera_id)
{
  if (m_camera != NULL) {
    CAM_INFO("camera %d already opened\n", camera_id);
    return true;
  }

  m_camera = new ArgusCameraContext(camera_id);

  return true;
}

bool CameraManager::closeCamera()
{
  deinitCamera();
  if (m_camera) {
    delete m_camera;
    m_camera = NULL;
  }

  return true;
}

bool CameraManager::initCamera()
{
  m_camera->createSession();

  return true;
}

void CameraManager::deinitCamera()
{
  m_camera->closeSession();
}

bool CameraManager::startCamera()
{
  m_camera->initCameraContext();

  return true;
}

bool CameraManager::stopCamera()
{
  m_camera->deinitCameraContext();

  return true;
}

int CameraManager::startPreview(int width, int height, std::string & usage)
{
  return m_camera->startPreview(width, height, usage);
}

int CameraManager::stopPreview()
{
  return m_camera->stopPreview();
}

void CameraManager::connSubCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    CAM_INFO("connect status : %d", msg->data);
    if (!msg->data) {
      CAM_INFO("recived app disconnect message, stop recording.");
      std::string filename;
      if (stopRecording(filename) == CAM_SUCCESS) {
        CAM_INFO("stopRecording success, file %s", filename.c_str());
        auto file_msg = std::make_unique<std_msgs::msg::String>();
        file_msg->data = filename;
        m_videoFilePub->publish(std::move(file_msg));
        if (m_videoFilePub.get()) {
          CAM_INFO("destroy video file name publisher");
          m_videoFilePub.reset();
        }
      }
    }
}

int CameraManager::startRecording(std::string & filename, int width, int height)
{
  int ret = CAM_SUCCESS;
  if (access(CAMERA_PATH, 0) != 0) {
    umask(0);
    mkdir(CAMERA_PATH, 0755);
  }

  filename = get_time_string() + ".mp4";
  ret = m_camera->startRecording(filename, width, height);
  if (ret == CAM_SUCCESS) {
    if (!m_connStateSub.get()) {
      CAM_INFO("create connect status subscription and file name publisher");
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      m_connStateSub = create_subscription<std_msgs::msg::Bool>("app_connection_state",
          qos, std::bind(&CameraManager::connSubCallback, this, std::placeholders::_1));
      m_videoFilePub = create_publisher<std_msgs::msg::String>("autosaved_video_file", qos);
    }
  }

  return ret;
}

int CameraManager::stopRecording(std::string & filename)
{
  if (m_connStateSub.get()) {
    CAM_INFO("destroy connect status subscription");
    m_connStateSub.reset();
  }
  return m_camera->stopRecording(filename);
}

bool CameraManager::isRecording()
{
  return m_camera->isRecording();
}

uint64_t CameraManager::getRecordingTime()
{
  return m_camera->getRecordingTime();
}

int CameraManager::takePicture(std::string & filename, int width, int height)
{
  /* create save directory if not exist.*/
  if (access(CAMERA_PATH, 0) != 0) {
    umask(0);
    mkdir(CAMERA_PATH, 0755);
  }

  filename = get_time_string();
  if (filename == m_lastPictureName) {
    filename = filename + "_" + std::to_string(++m_pictureIndex);
  } else {
    m_lastPictureName = filename;
    m_pictureIndex = 0;
  }

  filename += ".jpg";
  std::string path = CAMERA_PATH + filename;

  return m_camera->takePicture(path.c_str(), width, height);
}

int CameraManager::setParameters(std::string & parameters)
{
  std::map<std::string, std::string> params_map = parse_parameters(parameters);
  std::map<std::string, std::string>::iterator it;
  for (it = params_map.begin(); it != params_map.end(); it++) {
    setParameter(it->first, it->second);
  }

  return CAM_SUCCESS;
}

int CameraManager::setParameter(const std::string & key, const std::string & value)
{
  if (key == "face-interval") {
    printf("service: set face detect interval\n");
    if (atoi(value.c_str()) > 0) {
      m_camera->startRgbStream();
    } else {
      m_camera->stopRgbStream();
    }
  }

  return CAM_SUCCESS;
}

int CameraManager::startImagePublish(int width, int height, int rate)
{
  return m_camera->startImagePublish(width, height, rate);
}

int CameraManager::stopImagePublish()
{
  return m_camera->stopImagePublish();
}

}  // namespace camera
}  // namespace cyberdog
