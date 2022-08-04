cmake_minimum_required(VERSION 3.8)
project(camera_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_utils REQUIRED)
find_package(CUDA REQUIRED)
find_package(ARGUS)

if(NOT CUDA_FOUND)
  message(WARNING "CUDA NOT found.")
  return()
endif()

if(NOT ARGUS_FOUND)
  message(WARNING "ARGUS NOT found.")
  return()
endif()

# source code from argus api
set(ARGUS_INC_DIR ${MYSYSROOT}/usr/src/jetson_multimedia_api/argus/samples/utils
  ${MYSYSROOT}/usr/src/jetson_multimedia_api/include/libjpeg-8b
  ${MYSYSROOT}/usr/src/jetson_multimedia_api/include)
set(ARGUS_UTILS_DIR ${MYSYSROOT}/usr/src/jetson_multimedia_api/argus/samples/utils)
set(ARGUS_SRC_DIR ${MYSYSROOT}/usr/src/jetson_multimedia_api/samples/common/classes)

include_directories(include)
include_directories(${ARGUS_INCLUDE_DIR} ${ARGUS_INC_DIR})
link_directories(${MYSYSROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/tegra)

# ignore some compile warnings
add_compile_options(-Wno-pedantic
  -Wno-deprecated-declarations
  -Wno-unused-parameter
  -Wno-missing-field-initializers
)

cuda_add_library(color_convert
  src/cuda/CUDAConvert.cu
  src/cuda/CUDACrop.cu
)

set(ARGUS_SRCS
  ${ARGUS_SRC_DIR}/NvJpegEncoder.cpp
  ${ARGUS_SRC_DIR}/NvElement.cpp
  ${ARGUS_SRC_DIR}/NvElementProfiler.cpp
  ${ARGUS_SRC_DIR}/NvVideoEncoder.cpp
  ${ARGUS_SRC_DIR}/NvBuffer.cpp
  ${ARGUS_SRC_DIR}/NvV4l2Element.cpp
  ${ARGUS_SRC_DIR}/NvV4l2ElementPlane.cpp
  ${ARGUS_SRC_DIR}/NvLogging.cpp
  ${ARGUS_UTILS_DIR}/Thread.cpp
  ${ARGUS_UTILS_DIR}/NativeBuffer.cpp
  ${ARGUS_UTILS_DIR}/nvmmapi/NvNativeBuffer.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${ARGUS_SRCS}
  src/camera_dispatcher.cpp
  src/color_convert.cpp
  src/jpeg_encoder.cpp
  src/stream_consumer.cpp
  src/video_encoder.cpp
  src/video_muxer.cpp
)

target_link_libraries(${PROJECT_NAME} rt cuda EGL nvargus nvjpeg v4l2 avformat avcodec avutil pthread nvbuf_utils color_convert nvosd)
ament_target_dependencies(${PROJECT_NAME} rclcpp camera_utils)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
