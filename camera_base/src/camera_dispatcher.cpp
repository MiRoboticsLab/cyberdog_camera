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

#define LOG_TAG "CameraDispatcher"
#include <vector>
#include "camera_base/camera_dispatcher.hpp"
#include "camera_utils/camera_log.hpp"

namespace cyberdog
{
namespace camera
{

CameraDispatcher & CameraDispatcher::getInstance()
{
  static CameraDispatcher s_instance;

  s_instance.initialize();

  return s_instance;
}

CameraDispatcher::CameraDispatcher()
: m_initialized(false)
{
}

CameraDispatcher::~CameraDispatcher()
{
  if (!shutdown()) {
    CAM_ERR("Failed to shutdown");
  }
}

bool CameraDispatcher::initialize()
{
  if (m_initialized) {
    return true;
  }

  // Create the CameraProvider object.
  m_cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
  m_iCameraProvider =
    Argus::interface_cast<Argus::ICameraProvider>(m_cameraProvider);
  if (!m_iCameraProvider) {
    CAM_ERR("Failed to create CameraProvider");
    return false;
  }

  // Get the camera devices.
  m_iCameraProvider->getCameraDevices(&m_cameraDevices);
  if (m_cameraDevices.size() == 0) {
    shutdown();
    CAM_ERR("No cameras available");
    return false;
  }

  m_initialized = true;

  return true;
}

bool CameraDispatcher::shutdown()
{
  CAM_INFO("shutdown");
  if (m_initialized) {
    m_initialized = false;
    m_cameraDevices.clear();
    m_cameraProvider.reset();
  }

  return true;
}

bool CameraDispatcher::createSession(UniqueObj<CaptureSession> & session, uint32_t deviceIndex)
{
  assert(m_initialized);

  UniqueObj<CaptureSession> newSession(
    m_iCameraProvider->createCaptureSession(m_cameraDevices[deviceIndex]));
  if (!newSession) {
    CAM_ERR("Failed to create CaptureSession");
    return false;
  }

  session.reset(newSession.release());

  return true;
}

bool CameraDispatcher::createOutputStream(
  CaptureSession * session,
  UniqueObj<OutputStream> & stream, Size2D<uint32_t> size, Argus::StreamType type)
{
  if (!session) {
    CAM_ERR("NULL session");
    return false;
  }

  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  UniqueObj<Argus::OutputStreamSettings> outputStreamSettings(
    iCaptureSession->createOutputStreamSettings(type));

  if (type == Argus::STREAM_TYPE_BUFFER) {
    Argus::IBufferOutputStreamSettings * iOutputStreamSettings =
      Argus::interface_cast<Argus::IBufferOutputStreamSettings>(outputStreamSettings);
    if (!iOutputStreamSettings) {
      CAM_ERR("Failed to get IBufferOutputStreamSettings interface");
      return false;
    }

    /* Configure the OutputStream to use the EGLImage BufferType */
    iOutputStreamSettings->setBufferType(Argus::BUFFER_TYPE_EGL_IMAGE);
    iOutputStreamSettings->setMetadataEnable(true);
  } else if (type == Argus::STREAM_TYPE_EGL) {
    Argus::IEGLOutputStreamSettings * iEGLOutputStreamSettings =
      Argus::interface_cast<Argus::IEGLOutputStreamSettings>(outputStreamSettings);
    if (!iEGLOutputStreamSettings) {
      CAM_ERR("Failed to get IEGLOutputStreamSettings interface");
      return false;
    }

    iEGLOutputStreamSettings->setPixelFormat(Argus::PIXEL_FMT_YCbCr_420_888);
    iEGLOutputStreamSettings->setResolution(size);
    iEGLOutputStreamSettings->setMetadataEnable(true);
  }

  UniqueObj<OutputStream> outputStream(
    iCaptureSession->createOutputStream(outputStreamSettings.get()));
  if (!outputStream) {
    CAM_ERR("Failed to create OutputStream");
    return false;
  }

  stream.reset(outputStream.release());

  return true;
}

bool CameraDispatcher::createRequest(
  CaptureSession * session,
  UniqueObj<Request> & request, Argus::CaptureIntent captureIntent)
{
  if (!session) {
    CAM_ERR("NULL session");
    return false;
  }

  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  // Create request
  UniqueObj<Request> newRequest =
    UniqueObj<Request>(iCaptureSession->createRequest(captureIntent));
  if (!newRequest) {
    CAM_ERR("Failed to create request");
    return false;
  }

  // Setup request
  Argus::IRequest * iRequest = Argus::interface_cast<Argus::IRequest>(newRequest);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  // get source settings interface
  Argus::ISourceSettings * iSourceSettings =
    Argus::interface_cast<Argus::ISourceSettings>(iRequest->getSourceSettings());
  if (!iSourceSettings) {
    CAM_ERR("Failed to get ISourceSettings interface");
    return false;
  }

  iSourceSettings->setFrameDurationRange(Argus::Range<uint64_t>(1e9 / 30));

  request.reset(newRequest.release());

  return true;
}

bool CameraDispatcher::enableOutputStream(Request * request, OutputStream * stream)
{
  Argus::IRequest * iRequest = Argus::interface_cast<Argus::IRequest>(request);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  // Enable the stream
  if (iRequest->enableOutputStream(stream) != Argus::STATUS_OK) {
    CAM_ERR("Failed to enable the output stream");
    return false;
  }

  return true;
}

bool CameraDispatcher::disableOutputStream(Request * request, OutputStream * stream)
{
  Argus::IRequest * iRequest = Argus::interface_cast<Argus::IRequest>(request);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  // Enable the stream
  if (iRequest->disableOutputStream(stream) != Argus::STATUS_OK) {
    CAM_ERR("Failed to disable the output stream");
    return false;
  }

  return true;
}

bool CameraDispatcher::clearOutputStreams(Request * request)
{
  Argus::IRequest * iRequest = Argus::interface_cast<Argus::IRequest>(request);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  if (iRequest->clearOutputStreams() != Argus::STATUS_OK) {
    CAM_ERR("Failed to clear output streams");
    return false;
  }

  return true;
}

bool CameraDispatcher::getOutputStreams(Request * request, std::vector<OutputStream *> * streams)
{
  Argus::IRequest * iRequest = Argus::interface_cast<Argus::IRequest>(request);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  if (iRequest->getOutputStreams(streams) != Argus::STATUS_OK) {
    CAM_ERR("Failed to get output streams");
    return false;
  }

  return true;
}

bool CameraDispatcher::capture(CaptureSession * session, Request * request)
{
  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  if (iCaptureSession->capture(request, Argus::TIMEOUT_INFINITE) == 0) {
    CAM_ERR("Failed to submit the still capture request");
    return false;
  }

  return true;
}

bool CameraDispatcher::startRepeat(CaptureSession * session, Request * request)
{
  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  if (iCaptureSession->repeat(request) != Argus::STATUS_OK) {
    CAM_ERR("Failed to submit repeating capture request");
    return false;
  }

  return true;
}

bool CameraDispatcher::stopRepeat(CaptureSession * session)
{
  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  iCaptureSession->stopRepeat();

  return true;
}

bool CameraDispatcher::waitForIdle(CaptureSession * session)
{
  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  if (iCaptureSession->waitForIdle() != Argus::STATUS_OK) {
    CAM_ERR("Waiting for idle failed");
    return false;
  }

  return true;
}

bool CameraDispatcher::isRepeating(CaptureSession * session)
{
  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  return iCaptureSession->isRepeating();
}

bool CameraDispatcher::cancelRequests(CaptureSession * session)
{
  Argus::ICaptureSession * iCaptureSession =
    Argus::interface_cast<Argus::ICaptureSession>(session);
  if (!iCaptureSession) {
    CAM_ERR("Failed to get ICaptureSession interface");
    return false;
  }

  if (iCaptureSession->cancelRequests() != Argus::STATUS_OK) {
    CAM_ERR("Cancel requests failed");
    return false;
  }

  return true;
}

bool CameraDispatcher::getAllSensorModes(uint32_t deviceIndex,
  std::vector<SensorMode *> *modes)
{
  // Get the camera mode.
  Argus::ICameraProperties * iCameraProperties =
    Argus::interface_cast<Argus::ICameraProperties>(m_cameraDevices[deviceIndex]);
  iCameraProperties->getAllSensorModes(modes);

  return true;
}

Size2D<uint32_t> CameraDispatcher::getSensorSize(uint32_t deviceIndex)
{
  assert(m_cameraDevices.size() > deviceIndex);

  // Get the camera mode.
  std::vector<Argus::SensorMode *> sensorModes;
  getAllSensorModes(deviceIndex, &sensorModes);
  if (sensorModes.size() == 0) {
    CAM_ERR("No camera modes available");
    return Size2D<uint32_t>(0, 0);
  }

  Argus::ISensorMode * iSensorMode = Argus::interface_cast<Argus::ISensorMode>(sensorModes[0]);
  if (!iSensorMode) {
    return Size2D<uint32_t>(0, 0);
  }

  return iSensorMode->getResolution();
}

SensorMode * CameraDispatcher::getBestSensorMode(Size2D<uint32_t> size,
  std::vector<SensorMode *> &modes)
{
  if (modes.size() == 0)
    return nullptr;

  SensorMode *targetMode = modes[0];
  for (size_t i = 0; i < modes.size(); i++) {
    // find first mode with same resolution.
    Argus::ISensorMode * iSensorMode =
      Argus::interface_cast<Argus::ISensorMode>(modes[i]);
    Size2D<uint32_t> res = iSensorMode->getResolution();
    if (res.width() == size.width() && res.height() == size.height()) {
      targetMode = modes[i];
      CAM_INFO("Found best sensor mode %u with same resolution (%u/%u).",
          i, res.width(), res.height());
      return targetMode;
    }
  }

  for (size_t i = 0; i < modes.size(); i++) {
    // find first mode with same aspect ratio.
    Argus::ISensorMode * iSensorMode =
      Argus::interface_cast<Argus::ISensorMode>(modes[i]);
    Size2D<uint32_t> res = iSensorMode->getResolution();
    if ((res.width() * size.height()) == (size.width() * res.height())) {
      targetMode = modes[i];
      CAM_INFO("Found best sensor mode %u with same aspect ratio (%u/%u).",
          i, res.width(), res.height());
      return targetMode;
    }
  }

  CAM_INFO("best sensor mode not found, use default mode.");

  return targetMode;
}

SensorMode * CameraDispatcher::findBestSensorMode(uint32_t deviceIndex, Size2D<uint32_t> size)
{
  assert(m_cameraDevices.size() > deviceIndex);

  std::vector<Argus::SensorMode *> sensorModes;
  getAllSensorModes(deviceIndex, &sensorModes);
  if (sensorModes.size() == 0) {
    CAM_ERR("No camera modes available");
    return nullptr;
  }

  return getBestSensorMode(size, sensorModes);
}

SensorMode * CameraDispatcher::findBestSensorModeWithIds(uint32_t deviceIndex,
    Size2D<uint32_t> size,
    std::vector<uint32_t> &modeIds)
{
  assert(m_cameraDevices.size() > deviceIndex);

  std::vector<Argus::SensorMode *> sensorModes;
  getAllSensorModes(deviceIndex, &sensorModes);
  if (sensorModes.size() == 0) {
    CAM_ERR("No camera modes available");
    return nullptr;
  }

  for (auto it = modeIds.begin(); it < modeIds.end(); it++) {
    if (*it >= sensorModes.size()) {
      CAM_INFO("mode %u is invalid.", *it);
      modeIds.erase(it);
    }
  }

  if (modeIds.size() == 0) {
    CAM_INFO("No given modes, find the best in all modes.");
    return findBestSensorMode(deviceIndex, size);
  }

  std::vector<Argus::SensorMode *> filter_modes;
  for (auto it = modeIds.begin(); it < modeIds.end(); it++) {
    filter_modes.push_back(sensorModes[*it]);
  }

  return getBestSensorMode(size, filter_modes);
}

SensorMode * CameraDispatcher::getSensorMode(uint32_t deviceIndex, uint32_t modeIndex)
{
  assert(m_cameraDevices.size() > deviceIndex);

  std::vector<Argus::SensorMode *> sensorModes;
  getAllSensorModes(deviceIndex, &sensorModes);
  if (sensorModes.size() == 0) {
    CAM_ERR("No camera modes available");
    return nullptr;
  }

  if (modeIndex >= sensorModes.size()) {
    CAM_INFO("Sendoe Mode %u not existed, use default mode.", modeIndex);
    return sensorModes[0];
  } else {
    CAM_INFO("Sendoe Mode %u existed.", modeIndex);
    return sensorModes[modeIndex];
  }
}

bool CameraDispatcher::setSensorMode(Request * request, SensorMode * mode)
{
  Argus::IRequest * iRequest = Argus::interface_cast<Argus::IRequest>(request);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  Argus::ISourceSettings * iSourceSettings =
    Argus::interface_cast<Argus::ISourceSettings>(iRequest->getSourceSettings());
  if (!iSourceSettings) {
    CAM_ERR("Failed to get ISourceSettings interface");
    return false;
  }

  iSourceSettings->setSensorMode(mode);
  return true;
}

bool CameraDispatcher::setStreamClipRect(Request * request, OutputStream * stream, const Argus::Rectangle<float> &rect)
{
  Argus::IRequest *iRequest = Argus::interface_cast<Argus::IRequest>(request);
  if (!iRequest) {
    CAM_ERR("Failed to get IRequest interface");
    return false;
  }

  Argus::IStreamSettings *streamSettings = Argus::interface_cast<Argus::IStreamSettings>
        (iRequest->getStreamSettings(stream));
  streamSettings->setSourceClipRect(rect);
  Argus::Rectangle<float> r = streamSettings->getSourceClipRect();
  CAM_INFO("rect: (%f %f %f %f)", r.left(), r.top(), r.right(), r.bottom());

  return true;
}

}  // namespace camera
}  // namespace cyberdog
