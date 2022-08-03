// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "camera_api/camera_api.hpp"

int frame_callback(cv::Mat &frame, uint64_t ts,uint32_t frame_number, void * args)
{
  static int count = 0;
  RCLCPP_INFO(rclcpp::get_logger("camera_test"), "%s: ts - %llu", __FUNCTION__, ts);
  /*if (count++ == 30) {
    FILE * fp = fopen("rgb111.rgb", "wb+");

    fwrite(frame.data, 1, frame.cols * frame.rows * frame.channels(), fp);
    fclose(fp);
  }*/
  return 0;
}

int main(int argc, char * argv[])
{
  cyberdog::camera::CameraHandle cam_hdl = nullptr;

  if (argc != 5) {
    printf("usage: camera_test cam_id width height rgb/bgr\n");
    return 0;
  }

  int camera_id = atoi(argv[1]);
  int width = atoi(argv[2]);
  int height = atoi(argv[3]);
  cyberdog::camera::ImageFormat format;
  if (strcmp(argv[4], "bgr") == 0) {
    format =  cyberdog::camera::kImageFormatBGR;
  } else if (strcmp(argv[4], "rgb") == 0) {
    format =  cyberdog::camera::kImageFormatRGB;
  } else {
    printf("image format %s unsupported!\n", argv[4]);
    return 0;
  }

  int status;
  cam_hdl = cyberdog::camera::OpenCamera(camera_id, status);
  cyberdog::camera::StartStream(
    cam_hdl,
    format, width, height, frame_callback, NULL);

  sleep(300);

  cyberdog::camera::StopStream(cam_hdl);
  cyberdog::camera::CloseCamera(cam_hdl);
  return 0;
}
