/*
 * @Author: aoi
 * @Date: 2024-08-26 17:52:32
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-26 19:43:44
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#pragma once

#include <torch/script.h>

#include "./lcm/RealDesLcm.hpp"
#include "./lcm/lcm02_IO.h"
#include "rclcpp/rclcpp.hpp"

namespace humanoid {

class HumanoidControlInterface : public rclcpp::Node {
 public:
  HumanoidControlInterface(const rclcpp::NodeOptions &options);

  ~HumanoidControlInterface() = default;

  //
  bool init();
  void inference();
  void control();
  void observation();
  // void add_obs(const std::vector<double> &obs);

  std::shared_ptr<torch::jit::Module> model_rl_;
  std::deque<std::vector<double>> obs_history_;
  std::vector<double> action_;

  std::mutex mtx_inference_;  //
  std::mutex mtx_control_;    //
  std::mutex mtx_observation_;    //

  IOLcm *ioLcmInter_;

  std::chrono::high_resolution_clock::time_point start_time_;

  //
  rclcpp::TimerBase::SharedPtr inference_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr observation_timer_;

  rclcpp::CallbackGroup::SharedPtr humanoid_control_cb_group_;
};

}  // namespace humanoid