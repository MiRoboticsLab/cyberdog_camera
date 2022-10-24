cmake_minimum_required(VERSION 3.8)
project(camera_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_api REQUIRED)
find_package(camera_utils REQUIRED)
find_package(camera_service REQUIRED)
find_package(cyberdog_visions_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(protocol REQUIRED)
find_package(image_transmission)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
link_directories(${MYSYSROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/tegra)

add_compile_options(-Wno-unused-parameter)

add_executable(${PROJECT_NAME}
  src/camera_test.cpp
)

add_executable(camera_server
  src/camera_server.cpp
)

add_executable(stereo_camera
  src/dispatcher.cpp
  src/global_timestamp_reader.cpp
  src/stereo_camera.cpp
)

target_link_libraries(${PROJECT_NAME} cuda)
target_link_libraries(camera_server cuda)
target_link_libraries(stereo_camera cuda)
ament_target_dependencies(${PROJECT_NAME} camera_api OpenCV rclcpp protocol image_transmission)
ament_target_dependencies(camera_server camera_service rclcpp protocol image_transmission)
ament_target_dependencies(stereo_camera
    sensor_msgs camera_api camera_utils OpenCV protocol rclcpp image_transmission
    cyberdog_visions_interfaces lifecycle_msgs rclcpp_lifecycle)

install(TARGETS
  ${PROJECT_NAME}
  camera_server
  stereo_camera
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
