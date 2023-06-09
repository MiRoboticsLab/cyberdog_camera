cmake_minimum_required(VERSION 3.8)
project(camera_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(camera_base REQUIRED)
find_package(camera_utils REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

set(ARGUS_INC_DIR ${MYSYSROOT}/usr/src/jetson_multimedia_api/argus/samples/utils
  ${MYSYSROOT}/usr/src/jetson_multimedia_api/include)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${ARGUS_INC_DIR})

# ignore some compile warnings
add_compile_options(-Wno-pedantic
  -Wno-deprecated-declarations
  -Wno-unused-parameter
)

add_library(${PROJECT_NAME} SHARED
  src/rgb_stream.cpp
  src/mono_stream.cpp
  src/camera_api.cpp
)

ament_target_dependencies(${PROJECT_NAME} camera_base OpenCV rclcpp camera_utils CUDA)

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
