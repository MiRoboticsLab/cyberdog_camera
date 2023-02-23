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

#define LOG_TAG "MONOStream"
#include <string>
#include <memory>
#include <utility>
#include "mono_stream.hpp"
#include "camera_utils/camera_log.hpp"

namespace cyberdog
{
namespace camera
{

MonoStream::MonoStream(
    Size2D<uint32_t> size,
    ImageFormat format, FrameCallback cb, void * cb_args)
: StreamConsumer(size),
  format_(format),
  callback_(cb),
  cb_args_(cb_args)
{
}

MonoStream::~MonoStream()
{
}

bool MonoStream::threadInitialize()
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
  input_params.colorFormat = NvBufferColorFormat_GRAY8;

  if (NvBufferCreateEx(&gray_fd_, &input_params) < 0) {
    CAM_INFO("Failed to create NvBuffer.");
    return false;
  }

  return true;
}

bool MonoStream::threadShutdown()
{
  if (gray_fd_ > 0) {
    NvBufferDestroy(gray_fd_);
  }

  return StreamConsumer::threadShutdown();
}

bool MonoStream::processBuffer(Buffer * buffer)
{
  if (!buffer) {
    return false;
  }

  float frameRate;
  if (getFrameRate(frameRate)) {
    CAM_INFO("%.2f frames per second", frameRate);
  }

  uint64_t ts = 0;
  uint32_t capture_id = 0;
  IBuffer * iBuffer = Argus::interface_cast<IBuffer>(buffer);
  const Argus::CaptureMetadata * metadata = iBuffer->getMetadata();
  const Argus::ICaptureMetadata * iMetadata = Argus::interface_cast<const Argus::ICaptureMetadata>(
    metadata);
  if (iMetadata) {
    ts = iMetadata->getSensorTimestamp();
    capture_id = iMetadata->getCaptureId();
  }

  DmaBuffer * dma_buf = DmaBuffer::fromArgusBuffer(buffer);
  int fd = dma_buf->getFd();

  // Transform yuv to gray
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

  NvBufferTransform(fd, gray_fd_, &transform_params);

  if (callback_) {
    cv::Mat frame = cv::Mat(m_size.height(), m_size.width(), CV_8UC1, cv::Scalar(0, 0, 0));
    NvBuffer2Raw(gray_fd_, 0, frame.cols, frame.rows, frame.data);
    callback_(frame, ts,capture_id, cb_args_);
  }

  m_frameCount++;
  bufferDone(buffer);

  return true;
}

}  // namespace camera
}  // namespace cyberdog
