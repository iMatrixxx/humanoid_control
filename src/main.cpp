/*
 * @Author: aw
 * @Date: 2024-04-01 06:49:40
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-26 23:20:16
 * @Description:
 * Copyright (c) Air by aw, All Rights Reserved.
 */
#include <csignal>
#include <iostream>
#include <string>

#include "../include/humanoid_control_interface.h"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid::HumanoidControlInterface>(
      rclcpp::NodeOptions());
  if (!node->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("aw_robot"),
                 "initialize aw_robot_node failed!");
    return -1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}