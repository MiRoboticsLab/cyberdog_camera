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

#include "camera_utils/camera_log.hpp"
#include "camera_api/camera_api.hpp"
#include "camera_base/camera_dispatcher.hpp"
#include "camera_base/stream_consumer.hpp"
#include "rgb_stream.hpp"

namespace cyberdog
{
namespace camera
{

class CameraHolder
{
public:
  CameraHolder()
  {
  }
  ~CameraHolder()
  {
  }
  CameraHolder(const CameraHolder & other) = delete;
  CameraHolder(CameraHolder && other) = delete;

  bool initialize(int camera_id);
  bool shutdown();
  bool startStream(int width, int height, ImageFormat format, FrameCallback cb);
  bool stopStream();

private:
  Argus::UniqueObj<Argus::CaptureSession> m_captureSession;
  Argus::UniqueObj<Argus::Request> m_previewRequest;
  StreamConsumer * m_stream;
};

bool CameraHolder::initialize(int camera_id)
{
  if (m_captureSession) {
    return true;
  }

  if (!CameraDispatcher::getInstance().createSession(m_captureSession, camera_id)) {
    CAM_ERR("Failed to create CaptureSession");
    return false;
  }

  return true;
}

bool CameraHolder::shutdown()
{
  m_previewRequest.reset();
  m_captureSession.reset();

  return true;
}

bool CameraHolder::startStream(int width, int height,
    ImageFormat format, FrameCallback cb)
{
  m_stream = new RgbStream(Size2D<uint32_t>(width, height), format, cb);
  if (m_stream == NULL) {
    CAM_ERR("Failed to create stream.");
    return false;
  }

  Argus::UniqueObj<OutputStream> outputStream;
  if (!CameraDispatcher::getInstance().createOutputStream(
      m_captureSession.get(), outputStream, m_stream->getSize()))
  {
    CAM_ERR("Failed to create output stream");
    return false;
  }

  m_stream->setOutputStream(outputStream.release());
  m_stream->initialize();
  m_stream->waitRunning();

  if (!CameraDispatcher::getInstance().createRequest(
      m_captureSession.get(), m_previewRequest, Argus::CAPTURE_INTENT_PREVIEW))
  {
    CAM_ERR("Failed to create preview Request");
    return false;
  }

  CameraDispatcher::getInstance().enableOutputStream(
    m_previewRequest.get(), m_stream->getOutputStream());

  CAM_INFO("Starting repeat capture requests.\n");
  if (!CameraDispatcher::getInstance().startRepeat(
      m_captureSession.get(),
      m_previewRequest.get()))
  {
    CAM_ERR("Failed to start repeat capture request");
    return false;
  }

  return true;
}

bool CameraHolder::stopStream()
{
  CameraDispatcher::getInstance().stopRepeat(m_captureSession.get());
  CameraDispatcher::getInstance().waitForIdle(m_captureSession.get());
  CAM_INFO("Stop repeat capture requests.\n");

  CameraDispatcher::getInstance().disableOutputStream(
    m_previewRequest.get(), m_stream->getOutputStream());

  m_stream->endOfStream();
  m_stream->shutdown();
  delete m_stream;

  return true;
}

CameraHandle OpenCamera(int camera_id, int & status)
{
  CameraHandle handle = nullptr;

  CameraHolder * cam_holder = new CameraHolder();
  if (!cam_holder->initialize(camera_id)) {
    CAM_ERR("Failed to initialize camera %d", camera_id);
    delete cam_holder;
    cam_holder = nullptr;
  }

  if (cam_holder != nullptr) {
    handle = reinterpret_cast<CameraHandle>(cam_holder);
  }
  return handle;
}

int CloseCamera(CameraHandle handle)
{
  CameraHolder * cam_holder = nullptr;
  if (handle != nullptr) {
    cam_holder = reinterpret_cast<CameraHolder *>(handle);
    if (!cam_holder->shutdown()) {
      CAM_ERR("Failed to shutdown camera");
    }
    delete cam_holder;
  }

  return 0;
}

int StartStream(CameraHandle handle, ImageFormat format, int width, int height, FrameCallback cb)
{
  CameraHolder * cam_holder = nullptr;

  switch (format)
  {
    case kImageFormatBGR:
    case kImageFormatRGB:
      break;
    default:
      CAM_ERR("Unsupported image format %d", format);
      return false;
      break;
  }

  if (handle != nullptr) {
    cam_holder = reinterpret_cast<CameraHolder *>(handle);
    if (!cam_holder->startStream(width, height, format, cb)) {
      CAM_ERR("Failed to start camera stream");
    }
  }

  return 0;
}

int StopStream(CameraHandle handle)
{
  CameraHolder * cam_holder = nullptr;
  if (handle != nullptr) {
    cam_holder = reinterpret_cast<CameraHolder *>(handle);
    if (!cam_holder->stopStream()) {
      CAM_ERR("Failed to stop camera stream");
    }
  }
  return 0;
}

}  // namespace camera
}  // namespace cyberdog
