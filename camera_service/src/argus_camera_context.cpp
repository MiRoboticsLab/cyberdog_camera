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

#define LOG_TAG "CameraContext"

#include <string>
#include <memory>
#include <vector>
#include "camera_service/camera_manager.hpp"
#include "camera_service/argus_camera_context.hpp"
#include "camera_base/camera_dispatcher.hpp"
#include "camera_service/video_stream_consumer.hpp"
#include "camera_service/h264_stream_consumer.hpp"
#include "camera_service/rgb_stream_consumer.hpp"
#include "camera_service/algo_stream_consumer.hpp"
#include "camera_service/ncs_client.hpp"
#include "camera_utils/camera_log.hpp"
#include <EGLStream/NV/ImageNativeBuffer.h>
#include <NvJpegEncoder.h>

namespace cyberdog
{
namespace camera
{

const uint64_t ONE_SECONDS_IN_NANOSECONDS = 1000000000;
const int VIDEO_WIDTH_DEFAULT = 1280;
const int VIDEO_HEIGHT_DEFAULT = 960;
const int ALGO_WIDTH_DEFAULT = 640;
const int ALGO_HEIGHT_DEFAULT = 480;
const int IMAGE_WIDTH_DEFAULT = 640;
const int IMAGE_HEIGHT_DEFAULT = 480;
const int LIVE_WIDTH_DEFAULT = 1280;
const int LIVE_HEIGHT_DEFAULT = 720;

ArgusCameraContext::ArgusCameraContext(int camId)
: m_cameraId(camId),
  m_isStreaming(false),
  liveStream_(CameraManager::getInstance()->getParent()),
  live_stream_cb_(nullptr)
{
  initialize();
}

ArgusCameraContext::~ArgusCameraContext()
{
  shutdown();
}

bool ArgusCameraContext::initialize()
{
  CAM_INFO("%s", __FUNCTION__);

  for (int i = 0; i < STREAM_TYPE_MAX; i++) {
    m_activeStreams.push_back(NULL);
  }

  live_stream_cb_ = liveStream_.Init();
  if (!live_stream_cb_) {
    CAM_ERR("Failed to init live stream broadcaster.");
  }

  return true;
}

bool ArgusCameraContext::shutdown()
{
  CAM_INFO("%s", __FUNCTION__);

  if (m_captureSession) {
    closeSession();
  }

  return true;
}

bool ArgusCameraContext::createSession()
{
  CAM_INFO("%s", __FUNCTION__);

  return true;
}

bool ArgusCameraContext::closeSession()
{
  CAM_INFO("%s", __FUNCTION__);

  if (!m_captureSession) {
    return true;
  }

  deinitCameraContext();

  return true;
}

bool ArgusCameraContext::initCameraContext()
{
  return true;
}

bool ArgusCameraContext::deinitCameraContext()
{
  if (CameraDispatcher::getInstance().isRepeating(m_captureSession.get())) {
    CameraDispatcher::getInstance().stopRepeat(m_captureSession.get());
    CameraDispatcher::getInstance().waitForIdle(m_captureSession.get());
  }

  // Destroy the output stream to end the consumer thread,
  // and wait for the consumer thread to complete.
  for (unsigned i = 0; i < m_activeStreams.size(); i++) {
    if (m_activeStreams[i]) {
      m_activeStreams[i]->endOfStream();
      m_activeStreams[i]->shutdown();
      m_activeStreams[i] = NULL;
    }
  }

  NCSClient::getInstance().requestLed(false);
  m_previewRequest.reset();
  m_captureSession.reset();

  return true;
}

int ArgusCameraContext::takePicture(const char * path, int width, int height)
{
  if (!m_isStreaming) {
    CAM_ERR("Device is not stream on.");
    return CAM_INVALID_STATE;
  }

  UniqueObj<Argus::Request> captureRequest;
  if (!CameraDispatcher::getInstance().createRequest(
      m_captureSession.get(), captureRequest, Argus::CAPTURE_INTENT_STILL_CAPTURE))
  {
    CAM_ERR("Failed to create capture Request");
    return CAM_ERROR;
  }

  Size2D<uint32_t> captureSize;
  if (width == 0 || height == 0) {
    captureSize = getSensorSize();
  } else {
    captureSize = Size2D<uint32_t>(width, height);
  }

  UniqueObj<OutputStream> captureStream;
  if (!CameraDispatcher::getInstance().createOutputStream(
      m_captureSession.get(), captureStream, captureSize, Argus::STREAM_TYPE_EGL))
  {
    CAM_ERR("Failed to create capture output stream");
    return false;
  }

  if (!CameraDispatcher::getInstance().enableOutputStream(
      captureRequest.get(), captureStream.get()))
  {
    CAM_ERR("Failed to enable capture stream");
    return CAM_ERROR;
  }

  UniqueObj<EGLStream::FrameConsumer> consumer(
    EGLStream::FrameConsumer::create(captureStream.get()));
  EGLStream::IFrameConsumer * iFrameConsumer =
    Argus::interface_cast<EGLStream::IFrameConsumer>(consumer);
  if (!iFrameConsumer) {
    CAM_ERR("Failed to create FrameConsumer");
    return CAM_ERROR;
  }

  if (!CameraDispatcher::getInstance().capture(
      m_captureSession.get(), captureRequest.get()))
  {
    CAM_ERR("Failed to submit the still capture request");
    return CAM_ERROR;
  }

  UniqueObj<EGLStream::Frame> frame(iFrameConsumer->acquireFrame(ONE_SECONDS_IN_NANOSECONDS));
  if (!frame) {
    CAM_ERR("Failed to aquire frame");
    return CAM_ERROR;
  }

  // Use the IFrame interface to provide access to the Image in the Frame.
  EGLStream::IFrame * iFrame = Argus::interface_cast<EGLStream::IFrame>(frame);
  if (!iFrame) {
    CAM_ERR("Failed to get IFrame interface.");
    return CAM_ERROR;
  }

  EGLStream::NV::IImageNativeBuffer *iNativeBuffer =
      Argus::interface_cast<EGLStream::NV::IImageNativeBuffer>(iFrame->getImage());
  if (!iNativeBuffer) {
    CAM_ERR("IImageNativeBuffer not supported by Image.");
    return CAM_ERROR;
  }
  int fd = iNativeBuffer->createNvBuffer(captureSize,
          NvBufferColorFormat_NV12,
          NvBufferLayout_BlockLinear);
  if (fd == -1) {
    CAM_ERR("Failed to createNvBuffer.");
    return CAM_ERROR;
  }

  NvJPEGEncoder *jpegEncoder = NvJPEGEncoder::createJPEGEncoder("jpenenc");
  if (!jpegEncoder) {
    CAM_ERR("Failed to create JPEGEncoder.");
    NvBufferDestroy(fd);
    return CAM_ERROR;
  }

  int ret = CAM_SUCCESS;
  size_t jpeg_size = 5 * 1024 * 1024;
  uint8_t *jpeg_buf = new uint8_t[5 * 1024 * 1024];
  if (jpegEncoder->encodeFromFd(fd, JCS_YCbCr, &jpeg_buf, jpeg_size, 95) != 0) {
    CAM_ERR("Failed to encode buffer to jpeg.");
    ret = CAM_ERROR;
  }

  if (ret == CAM_SUCCESS) {
    std::ofstream outputFile = std::ofstream(path);
    outputFile.write((char *)jpeg_buf, jpeg_size);
    if (outputFile.tellp() != (int)jpeg_size) {
      CAM_ERR("Failed to write jpeg file, maybe no space.");
      ret = CAM_ERROR;
      remove_file(path);
    }
    outputFile.close();
  }

  NCSClient::getInstance().play(SoundShutter);
  NvBufferDestroy(fd);
  delete jpeg_buf;
  delete jpegEncoder;

  return ret;
}

int ArgusCameraContext::startCameraStream(
  Streamtype type,
  Size2D<uint32_t> size, void * args)
{
  std::lock_guard<std::mutex> lock(m_streamLock);
  std::string * filename;
  std::shared_ptr<StreamConsumer> stream;

  if (size.width() == 0 || size.height() == 0) {
    CAM_ERR("Stream size required is illegal - (%u:%u)", size.width(), size.height());
    return CAM_INVALID_ARG;
  }

  if (m_activeStreams[type]) {
    return CAM_INVALID_STATE;
  }

  if (!m_captureSession) {
    if (!CameraDispatcher::getInstance().createSession(m_captureSession, m_cameraId)) {
      CAM_ERR("Failed to create CaptureSession");
      return false;
    }
  }

  if (!m_previewRequest) {
    if (!CameraDispatcher::getInstance().createRequest(
        m_captureSession.get(), m_previewRequest, Argus::CAPTURE_INTENT_PREVIEW))
    {
      CAM_ERR("Failed to create preview Request");
      return false;
    }
  }

  if (CameraDispatcher::getInstance().isRepeating(m_captureSession.get())) {
    CameraDispatcher::getInstance().stopRepeat(m_captureSession.get());
    CameraDispatcher::getInstance().waitForIdle(m_captureSession.get());
  }

  switch (type) {
    case STREAM_TYPE_ALGO:
      stream = std::make_shared<AlgoStreamConsumer>(size);
      break;
    case STREAM_TYPE_RGB:
      if (args != NULL) {
        stream = std::make_shared<RGBStreamConsumer>(size, *((int *)args));
      } else {
        stream = std::make_shared<RGBStreamConsumer>(size, 0);
      }
      break;
    case STREAM_TYPE_VIDEO:
      filename = static_cast<std::string *>(args);
      CAM_INFO("filename = %s", filename->c_str());
      stream = std::make_shared<VideoStreamConsumer>(size, *filename);
      break;
    case STREAM_TYPE_H264:
      stream = std::make_shared<H264StreamConsumer>(size, live_stream_cb_);
      break;
    default:
      return CAM_INVALID_ARG;
  }

  UniqueObj<OutputStream> outputStream;
  if (!CameraDispatcher::getInstance().createOutputStream(
      m_captureSession.get(), outputStream, stream->getSize()))
  {
    CAM_ERR("Failed to create output stream %d", type);
    return CAM_ERROR;
  }
  stream->setOutputStream(outputStream.release());
  stream->initialize();
  stream->waitRunning();

  //图传可以选择居中、顶部、底部裁切
  if (type == STREAM_TYPE_H264) {
    std::string * position = static_cast<std::string *>(args);
    float crop_ratio = 1.f - (4.f * size.height()) / (3.f * size.width());

    //默认是居中裁切
    Argus::Rectangle<float> clip_rect = {0, crop_ratio / 2, 1, (1 - crop_ratio / 2)};
    if (position != NULL) {
      if (*position == "top") {
        clip_rect = {0, 0, 1, (1 - crop_ratio)};
      } else if (*position ==  "bottom") {
        clip_rect = {0, crop_ratio, 1, 1};
      }
    }
    CameraDispatcher::getInstance().setStreamClipRect(m_previewRequest.get(),
              stream->getOutputStream(), clip_rect);
  }

  m_activeStreams[type] = stream;
  CameraDispatcher::getInstance().enableOutputStream(
    m_previewRequest.get(), m_activeStreams[type]->getOutputStream());

  std::vector<OutputStream *> streams;
  CameraDispatcher::getInstance().getOutputStreams(m_previewRequest.get(), &streams);
  if (streams.size() > 0) {
    // Submit capture requests.
    CAM_INFO("Starting repeat capture requests.\n");
    if (!CameraDispatcher::getInstance().startRepeat(
        m_captureSession.get(),
        m_previewRequest.get()))
    {
      CAM_ERR("Failed to start repeat capture request");
      return false;
    }
    m_isStreaming = true;
    NCSClient::getInstance().requestLed(true);
  }
  return CAM_SUCCESS;
}

int ArgusCameraContext::stopCameraStream(Streamtype type)
{
  std::lock_guard<std::mutex> lock(m_streamLock);
  if (!m_activeStreams[type]) {
    return CAM_INVALID_STATE;
  }

  if (CameraDispatcher::getInstance().isRepeating(m_captureSession.get())) {
    CameraDispatcher::getInstance().stopRepeat(m_captureSession.get());
    CameraDispatcher::getInstance().waitForIdle(m_captureSession.get());
  }
  CameraDispatcher::getInstance().disableOutputStream(
    m_previewRequest.get(), m_activeStreams[type]->getOutputStream());

  m_activeStreams[type]->endOfStream();
  m_activeStreams[type]->shutdown();
  m_activeStreams[type] = NULL;

  std::vector<OutputStream *> streams;
  CameraDispatcher::getInstance().getOutputStreams(m_previewRequest.get(), &streams);
  if (streams.size() > 0) {
    // Submit capture requests.
    CAM_INFO("Starting repeat capture requests.\n");
    if (!CameraDispatcher::getInstance().startRepeat(
        m_captureSession.get(),
        m_previewRequest.get()))
    {
      CAM_ERR("Failed to start repeat capture request");
      return false;
    }
  } else {
    m_isStreaming = false;
    NCSClient::getInstance().requestLed(false);
    m_previewRequest.reset();
    m_captureSession.reset();
  }

  return CAM_SUCCESS;
}

int ArgusCameraContext::startPreview(int width, int height, std::string & usage)
{
  int ret = CAM_SUCCESS;

  if (width == 0 || height == 0) {
    CAM_INFO("preview size not set, use default");
    width = LIVE_WIDTH_DEFAULT;
    height = LIVE_HEIGHT_DEFAULT;
  }
  ret = startCameraStream(
    STREAM_TYPE_H264,
    Size2D<uint32_t>(width, height), &usage);

  if (ret == CAM_SUCCESS/* && usage == "preview"*/) {
    NCSClient::getInstance().play(SoundLiveStart);
  }
  if (ret == CAM_INVALID_STATE) {
    ret = CAM_SUCCESS;
  }

  return ret;
}

int ArgusCameraContext::stopPreview()
{
  return stopCameraStream(STREAM_TYPE_H264);
}

int ArgusCameraContext::startRecording(std::string & filename, int width, int height)
{
  int ret = CAM_SUCCESS;

  if (width == 0 || height == 0) {
    width = VIDEO_WIDTH_DEFAULT;
    height = VIDEO_HEIGHT_DEFAULT;
  }

  ret = startCameraStream(
    STREAM_TYPE_VIDEO,
    Size2D<uint32_t>(width, height), &filename);

  if (ret == CAM_SUCCESS) {
    NCSClient::getInstance().play(SoundRecordStart);
    m_videoFilename = filename;
  }

  return ret;
}

int ArgusCameraContext::stopRecording(std::string & filename)
{
  int ret = stopCameraStream(STREAM_TYPE_VIDEO);
  if (ret == CAM_SUCCESS) {
    filename = m_videoFilename;
  }

  return ret;
}

int ArgusCameraContext::startRgbStream()
{
  return startCameraStream(
      STREAM_TYPE_ALGO, Size2D<uint32_t>(
        ALGO_WIDTH_DEFAULT,
        ALGO_HEIGHT_DEFAULT), NULL);
}

int ArgusCameraContext::stopRgbStream()
{
  return stopCameraStream(STREAM_TYPE_ALGO);
}

bool ArgusCameraContext::isRecording()
{
  return !!m_activeStreams[STREAM_TYPE_VIDEO];
}

uint64_t ArgusCameraContext::getRecordingTime()
{
  if (!m_activeStreams[STREAM_TYPE_VIDEO]) {
    return 0;
  }
  VideoStreamConsumer * video_stream =
    reinterpret_cast<VideoStreamConsumer *>(m_activeStreams[STREAM_TYPE_VIDEO].get());

  return video_stream->getRecordingTime();
}

int ArgusCameraContext::startImagePublish(int width, int height, int rate)
{
  return startCameraStream(
      STREAM_TYPE_RGB, Size2D<uint32_t>(
        width,
        height), &rate);
}

int ArgusCameraContext::stopImagePublish()
{
  int ret = stopCameraStream(STREAM_TYPE_RGB);

  //for image publish, return success if already started.
  if (ret == CAM_INVALID_STATE) {
    ret = CAM_SUCCESS;
  }

  return ret;
}

Size2D<uint32_t> ArgusCameraContext::getSensorSize()
{
  return CameraDispatcher::getInstance().getSensorSize(m_cameraId);
}

}  // namespace camera
}  // namespace cyberdog
