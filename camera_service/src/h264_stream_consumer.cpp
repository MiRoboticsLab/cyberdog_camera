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

#define LOG_TAG "H264Stream"
#include <string>
#include <memory>
#include <vector>
#include <utility>
#include "camera_service/h264_stream_consumer.hpp"
#include "camera_service/camera_manager.hpp"
#include "camera_service/ncs_client.hpp"
#include "camera_utils/camera_log.hpp"

#define USE_SOFT_ENC 1

namespace cyberdog
{
namespace camera
{

H264StreamConsumer::H264StreamConsumer(Size2D<uint32_t> size, StreamCb callback)
: StreamConsumer(size),
  live_stream_cb_(callback)
{
}

H264StreamConsumer::~H264StreamConsumer()
{
}

bool H264StreamConsumer::threadInitialize()
{
  if (!StreamConsumer::threadInitialize()) {
    return false;
  }

  NvBufferCreateParams input_params;
  input_params.width = m_size.width();
  input_params.height = m_size.height();
  input_params.payloadType = NvBufferPayload_SurfArray;
  input_params.nvbuf_tag = NvBufferTag_NONE;
  input_params.layout = NvBufferLayout_Pitch;
  input_params.colorFormat = NvBufferColorFormat_YUV420;

  if (NvBufferCreateEx(&i420_fd_, &input_params) < 0) {
    CAM_INFO("Failed to create NvBuffer.");
    return false;
  }

  return true;
}

bool H264StreamConsumer::threadShutdown()
{
  if (i420_fd_ > 0) {
    NvBufferDestroy(i420_fd_);
  }

  return StreamConsumer::threadShutdown();
}

bool H264StreamConsumer::processBuffer(Buffer * buffer)
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  if (!buffer) {
    return false;
  }

  float frameRate;
  if (getFrameRate(frameRate)) {
    CAM_INFO("%.2f frames per second", frameRate);
  }

  DmaBuffer * dma_buf = DmaBuffer::fromArgusBuffer(buffer);
  int fd = dma_buf->getFd();

  NvBufferTransformParams transform_params;
  NvBufferRect src_rect, dest_rect;
  memset(&transform_params, 0, sizeof(transform_params));

  src_rect.top = 0;
  src_rect.left = 0;
  src_rect.width = m_size.width();
  src_rect.height = m_size.height();
  dest_rect.top = 0;
  dest_rect.left = 0;
  dest_rect.width = m_size.width();
  dest_rect.height = m_size.height();
  transform_params.transform_flag = NVBUFFER_TRANSFORM_FILTER;
  transform_params.transform_flip = NvBufferTransform_None;
  transform_params.transform_filter = NvBufferTransform_Filter_Nicest;
  transform_params.src_rect = src_rect;
  transform_params.dst_rect = dest_rect;

  NvBufferTransform(fd, i420_fd_, &transform_params);

#ifdef USE_SOFT_ENC
  ImageBuffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.res = m_size;
  buf.fd = fd;
  buf.timestamp = ts;

  publishImage(m_frameCount, buf);
#endif
  bufferDone(buffer);

  m_frameCount++;

  return true;
}

void H264StreamConsumer::publishImage(uint64_t frame_id, ImageBuffer & buf)
{
#ifdef USE_SOFT_ENC
  int height = buf.res.height();
  int width = buf.res.width();
  int64_t size = height * width * 3 / 2;
  uint8_t * data = new uint8_t[size];
  memset(data, 0, size * sizeof(uint8_t));
  NvBuffer2Raw(i420_fd_, 0, width, height, &data[0]);
  NvBuffer2Raw(
    i420_fd_, 1, width / 2, height / 2,
    &data[width * height]);
  NvBuffer2Raw(
    i420_fd_, 2, width / 2, height / 2,
    &data[width * height + width * height / 4]);
  int64_t timestamp =
      buf.timestamp.tv_sec * 1000 * 1000  * 1000 + buf.timestamp.tv_nsec;
  if (live_stream_cb_) {
    live_stream_cb_(data, timestamp, frame_id, height, width);
  }
  delete[] data;
#endif
}

bool H264StreamConsumer::startSoundTimer()
{
  struct sigevent evp;
  struct itimerspec ts;
  memset(&evp, 0, sizeof(struct sigevent));
  memset(&ts, 0, sizeof(struct itimerspec));

  evp.sigev_value.sival_ptr = this;
  evp.sigev_notify = SIGEV_THREAD;
  evp.sigev_notify_function = playVideoSound;
  if (timer_create(CLOCK_MONOTONIC, &evp, &m_timer) != 0) {
    CAM_ERR("create sound playing timer failed.");
    return false;
  }

  ts.it_value.tv_sec = 1;
  ts.it_value.tv_nsec = 0;
  ts.it_interval.tv_sec = 120;
  ts.it_interval.tv_nsec = 0;
  if (timer_settime(m_timer, 0, &ts, NULL) != 0) {
    CAM_ERR("start sound playing timer failed.");
    return false;
  }
  CAM_INFO("Start preview sound timer.");

  return true;
}

void H264StreamConsumer::stopSoundTimer()
{
  CAM_INFO("Stop preview sound timer.");
  timer_delete(m_timer);
}

void H264StreamConsumer::playVideoSound(union sigval val)
{
  (void)val;
  NCSClient::getInstance().play(SoundRecording);
}

}  // namespace camera
}  // namespace cyberdog
