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

#define LOG_TAG "AlgoStream"
#include "camera_service/algo_stream_consumer.hpp"
#include "camera_utils/camera_log.hpp"
#include "camera_algo/algo_dispatcher.hpp"
#include "camera_service/shared_memory_op.hpp"
#include "camera_service/semaphore_op.hpp"

#define SHM_PROJ_ID 'A'
#define SEM_PROJ_ID 'B'

namespace cyberdog
{
namespace camera
{

AlgoStreamConsumer::AlgoStreamConsumer(Size2D<uint32_t> size)
: StreamConsumer(size)
, shm_addr_(nullptr)
{
  initSharedBuffer();
  global_time_.enable_time_diff_keeper(true);
}

AlgoStreamConsumer::~AlgoStreamConsumer()
{
  freeBuffers();
  deinitSharedBuffer();
}

ImageBuffer AlgoStreamConsumer::getBuffer()
{
  std::lock_guard<std::mutex> lock(m_bufferMutex);
  ImageBuffer buffer;
  for (auto it = m_freeBuffers.begin(); it != m_freeBuffers.end(); it++) {
    buffer = *it;
    m_freeBuffers.erase(it);
    return buffer;
  }
  CAM_INFO("get a new buffer");
  size_t size = m_size.width() * m_size.height() * 3;
  buffer.data = malloc(size);

  return buffer;
}

void AlgoStreamConsumer::putBuffer(ImageBuffer buffer)
{
  std::lock_guard<std::mutex> lock(m_bufferMutex);
  m_freeBuffers.push_back(buffer);
}

void AlgoStreamConsumer::freeBuffers()
{
  std::lock_guard<std::mutex> lock(m_bufferMutex);
  for (auto it = m_freeBuffers.begin(); it != m_freeBuffers.end(); it++) {
    auto buffer = *it;
    CAM_INFO("free a buffer");
    free(buffer.data);
  }
  m_freeBuffers.clear();
}

bool AlgoStreamConsumer::threadInitialize()
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
  input_params.colorFormat = NvBufferColorFormat_ABGR32;

  if (NvBufferCreateEx(&m_rgbaFd, &input_params) < 0) {
    CAM_ERR("Failed to create NvBuffer.");
    return false;
  }

  m_convert = new ColorConvert(m_size.width(), m_size.height());
  m_convert->initialze(m_rgbaFd);

  AlgoDispatcher::getInstance().setBufferDoneCallback(bufferDoneCallback, this);

  return true;
}

bool AlgoStreamConsumer::threadShutdown()
{
  m_convert->release();
  delete m_convert;

  if (m_rgbaFd > 0) {
    NvBufferDestroy(m_rgbaFd);
  }

  AlgoDispatcher::getInstance().setAlgoEnabled(ALGO_FACE_DETECT, false);
  AlgoDispatcher::getInstance().setAlgoEnabled(ALGO_BODY_DETECT, false);

  return StreamConsumer::threadShutdown();
}

double AlgoStreamConsumer::get_frame_timestamp(double frame_time)
{
  auto sp = global_time_._tf_keeper;
  bool ts_is_ready;
  if (sp)
      frame_time = sp->get_system_hw_time(frame_time, ts_is_ready);
  else
      CAM_ERR("Notification: global_timestamp_reader - time_diff_keeper is being shut-down");

  return frame_time;
}

bool AlgoStreamConsumer::processBuffer(Buffer * buffer)
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

  uint64_t hw_ts = 0, aligned_ts = 0;
  double real_ts_sys;
  IBuffer * iBuffer = Argus::interface_cast<IBuffer>(buffer);
  const Argus::CaptureMetadata * metadata = iBuffer->getMetadata();
  const Argus::ICaptureMetadata * iMetadata =
      Argus::interface_cast<const Argus::ICaptureMetadata>(metadata);
  if (iMetadata) {
    hw_ts = iMetadata->getSensorTimestamp();
    real_ts_sys = get_frame_timestamp(hw_ts * 0.000001)*0.001;
    aligned_ts = (uint64_t)(real_ts_sys * 1000 * 1000 * 1000);
  }

  DmaBuffer * dma_buf = DmaBuffer::fromArgusBuffer(buffer);
  int fd = dma_buf->getFd();
  ImageBuffer buf = getBuffer();
  buf.res = m_size;
  buf.timestamp = ts;
  buf.fd = fd;

  if (AlgoDispatcher::getInstance().needProcess(m_frameCount)) {
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

    NvBufferTransform(fd, m_rgbaFd, &transform_params);
    // m_convert->convertRGBAToBGR(buf.data);
    if (shm_addr_ != nullptr) {
      int ret = WaitSem(sem_set_id_, 1);
      if (ret == 0) {
        WaitSem(sem_set_id_, 0);
        memcpy(shm_addr_, &aligned_ts, sizeof(uint64_t));
        m_convert->convertRGBAToBGR(shm_addr_ + sizeof(uint64_t));
        SignalSem(sem_set_id_, 0);
        SignalSem(sem_set_id_, 2);
      }
    }

    AlgoDispatcher::getInstance().processImageBuffer(m_frameCount, buf);
  } else {
    imageBufferDone(buf);
  }

  m_frameCount++;

  return true;
}

void AlgoStreamConsumer::imageBufferDone(ImageBuffer buffer)
{
  int fd = buffer.fd;
  Argus::Buffer * dma_buffer = getBufferFromFd(fd);

  if (dma_buffer) {
    bufferDone(dma_buffer);
  }

  putBuffer(buffer);
}

bool AlgoStreamConsumer::initSharedBuffer()
{
  if (0 != CreateShm(SHM_PROJ_ID, sizeof(uint64_t) + IMAGE_SIZE, shm_id_)) {
    CAM_ERR("Failed to create shared memory.");
    return false;
  }
  shm_addr_ = GetShmAddr(shm_id_, sizeof(uint64_t) + IMAGE_SIZE);
  if (shm_addr_ == nullptr) {
    CAM_ERR("Failed to map shared memory.");
    return false;
  }
  if (0 != CreateSem(SEM_PROJ_ID, 3, sem_set_id_)) {
    CAM_ERR("Failed to create shared memory semaphore.");
    return false;
  }

  return true;
}

void AlgoStreamConsumer::deinitSharedBuffer()
{
  DetachShm(shm_addr_);
}

}  // namespace camera
}  // namespace cyberdog
