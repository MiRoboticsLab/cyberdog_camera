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

#ifndef CAMERA_SERVICE__MAIN_CAMERA_NODE_HPP_
#define CAMERA_SERVICE__MAIN_CAMERA_NODE_HPP_

#include <string>
#include <memory>
#include "ros2_service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <image_transmission/live_stream_broadcaster.hpp>

namespace cyberdog
{
namespace camera
{

class CameraServerNode : public rclcpp::Node
{
public:
  CameraServerNode();
  virtual ~CameraServerNode();

protected:
  void on_configure() ;
  void on_activate() ;
  void on_deactivate() ;
  void on_cleanup() ;
  void on_shutdown() ;

private:
  void serviceCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<CameraServiceT::Request> request,
    std::shared_ptr<CameraServiceT::Response> response);
  void initParameters();

  int takePicture(std::string & args, std::shared_ptr<CameraServiceT::Response> response);
  int startRecording(std::string &, std::shared_ptr<CameraServiceT::Response> response);
  int stopRecording(std::shared_ptr<CameraServiceT::Response> response);
  int getState(std::shared_ptr<CameraServiceT::Response> response);
  int deleteFile(const std::string & filename);
  int getAllFiles(std::shared_ptr<CameraServiceT::Response> response);

  size_t width_;
  size_t height_;

  //  camera service
  rclcpp::Service<CameraServiceT>::SharedPtr m_camService;
  //  face manager service
  rclcpp::Service<FaceManagerT>::SharedPtr m_faceManager;
};

}  // namespace camera
}  // namespace cyberdog

#endif  // CAMERA_SERVICE__MAIN_CAMERA_NODE_HPP__
