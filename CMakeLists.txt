cmake_minimum_required(VERSION 3.8)
project(humanoid_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
message("CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
set(Torch_DIR ${CMAKE_SOURCE_DIR}/libtorch/share/cmake/Torch)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Torch REQUIRED)

set(src_files
  "src/lcm02_IO.cpp"
  "src/humanoid_control_interface.cpp"
  "src/main.cpp"
)
include_directories(
  include 
  ${TORCH_INCLUDE_DIRS}  # LibTorch的包含目录
)


add_executable(${PROJECT_NAME}_node ${src_files})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
target_link_libraries(${PROJECT_NAME}_node pthread "${TORCH_LIBRARIES}" lcm)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()