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

#ifndef CAMERA_SERVICE__ROS2_SERVICE_HPP_
#define CAMERA_SERVICE__ROS2_SERVICE_HPP_

/*
 * Camera ros2 service interfaces
 */

#include "protocol/srv/camera_service.hpp"
#include "protocol/srv/face_manager.hpp"
#include "protocol/msg/body_info.hpp"
#include "protocol/msg/face_info.hpp"
#include "protocol/msg/face_result.hpp"
// #include "protocol/action/audio_play.hpp"
// #include "ception_msgs/srv/sensor_detection_node.hpp"

using CameraServiceT = protocol::srv::CameraService;
using FaceManagerT = protocol::srv::FaceManager;
using BodyInfoT = protocol::msg::BodyInfo;
using BodyT = protocol::msg::Body;
using FaceInfoT = protocol::msg::FaceInfo;
using FaceT = protocol::msg::Face;
using FaceResultT = protocol::msg::FaceResult;
// using AudioPlayT = protocol::action::AudioPlay;
// using LedServiceT = ception_msgs::srv::SensorDetectionNode;

#endif  // CAMERA_SERVICE__ROS2_SERVICE_HPP_
