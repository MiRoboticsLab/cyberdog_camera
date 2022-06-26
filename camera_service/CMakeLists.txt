cmake_minimum_required(VERSION 3.8)
project(camera_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(protocol REQUIRED)
find_package(camera_base REQUIRED)
find_package(camera_utils REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transmission)

set(ARGUS_INC_DIR ${MYSYSROOT}/usr/src/jetson_multimedia_api/argus/samples/utils
  ${MYSYSROOT}/usr/src/jetson_multimedia_api/include)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${ARGUS_INC_DIR})
link_directories(${MYSYSROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/tegra)

# ignore some compile warnings
add_compile_options(-Wno-pedantic
  -Wno-deprecated-declarations
  -Wno-unused-parameter
)

add_library(${PROJECT_NAME} SHARED
  src/camera_algo/algo_dispatcher.cpp
  src/camera_algo/algorithm_base.cpp
  src/camera_algo/body_detect.cpp
  src/camera_algo/body_detector.cpp
  src/camera_algo/face_detect.cpp
  src/camera_algo/face_detector.cpp
  src/camera_algo/feature_detector.cpp
  src/algo_stream_consumer.cpp
  src/argus_camera_context.cpp
  src/camera_manager.cpp
  src/face_manager.cpp
  src/face_manager_node.cpp
  src/h264_stream_consumer.cpp
  src/main_camera_node.cpp
  src/ncs_client_dummy.cpp
  src/rgb_stream_consumer.cpp
  src/video_stream_consumer.cpp
)

target_link_libraries(${PROJECT_NAME} cuda nvosd)
ament_target_dependencies(${PROJECT_NAME}
  camera_base
  rclcpp
  rclcpp_action
  sensor_msgs
  protocol
  camera_utils
  CUDA
  OpenCV
  image_transmission
)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  )
install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
