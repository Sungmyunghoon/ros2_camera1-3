#최소 Cmake버전은 3.16으로 설정
cmake_minimum_required(VERSION 3.16)
#프로젝트 이름
project(camera1_3)

# Default to C++14
# C++ 표준의 기본값을 C++14로 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일러가 GNU C++ 또는 Clang인 경우, 추가 컴파일 옵션을 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 필요한 패키지를 찾는다
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

#build
add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(sub_wsl src/sub_wsl.cpp)
ament_target_dependencies(sub_wsl rclcpp sensor_msgs OpenCV)

# Install
install(TARGETS
  pub
  sub_wsl
  DESTINATION lib/${PROJECT_NAME})

ament_package()
