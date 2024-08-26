/*
 * @Author: aoi
 * @Date: 2024-08-26 17:57:02
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-26 19:55:45
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#include "../include/humanoid_control_interface.h"

#include <cmath>
#include <iostream>

using namespace std::chrono_literals;
using namespace humanoid;
HumanoidControlInterface::HumanoidControlInterface(
    const rclcpp::NodeOptions& options)
    : Node("humanoid_control_node", options) {}

bool HumanoidControlInterface::init() {  //
  obs_history_.resize(0);
  for (int i = 0; i < 15; ++i) {
    std::vector<double> new_vector(47);
    obs_history_.push_back(new_vector);
  }
  action_.resize(0);
  for (int i = 0; i < 12; i++) {
    action_.push_back(0.0);
  }
  try {
    // 使用 std::make_shared 包装返回的 torch::jit::Module
    model_rl_ = std::make_shared<torch::jit::Module>(torch::jit::load(
        "/home/fudanrobotuser/humanoid_ws/src/humanoid_control/model/model_rl.pt"));
  } catch (const c10::Error& e) {
    std::cerr << "Error loading the model: " << e.what() << std::endl;
    return false;
  }

  //
  humanoid_control_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  inference_timer_ = this->create_wall_timer(
      10ms, std::bind(&HumanoidControlInterface::inference, this),
      humanoid_control_cb_group_);

  control_timer_ = this->create_wall_timer(
      1ms, std::bind(&HumanoidControlInterface::control, this),
      humanoid_control_cb_group_);

  observation_timer_ = this->create_wall_timer(
      10ms, std::bind(&HumanoidControlInterface::observation, this),
      humanoid_control_cb_group_);

  ioLcmInter_ = new IOLcm("fudan_humanoid");

  return true;
}

void HumanoidControlInterface::inference() {
  std::lock_guard<std::mutex> lock(mtx_inference_);
  std::vector<float> input_data;
  for (auto& vector : obs_history_) {
    for (auto& element : vector) {
      input_data.push_back(element);
    }
  }
  torch::Tensor input_tensor = torch::from_blob(input_data.data(), {1, 705});

  // 执行推理
  auto action = model_rl_->forward({input_tensor}).toTensor();
  for (int i = 0; i < 12; i++) {
    action_[i] = action.index({0, i}).item<double>();
  }

  // std::cout << action_ << std::endl;
}

void HumanoidControlInterface::control() {
  std::lock_guard<std::mutex> lock(mtx_control_);

  // 发送action_ pd 调节
  ioLcmInter_->sendOutput(action_);
}

void HumanoidControlInterface::observation() {
  std::lock_guard<std::mutex> lock(mtx_observation_);
    // create obs
    // self.command_input,  # 5 = 2D(sin cos) + 3D(vel_x, vel_y, aug_vel_yaw)
    // q,    # 12D
    // dq,  # 12D
    // self.actions,   # 12D
    // self.base_ang_vel * self.obs_scales.ang_vel,  # 3
    // self.base_euler_xyz * self.obs_scales.quat * 0.0,  # 3
    std::vector<double> obs;  //
    // 记录结束时间点
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end - start_time_);

    obs.push_back(sin(2.0 * M_PI * duration.count() / 500.0));
    obs.push_back(cos(2.0 * M_PI * duration.count() / 500.0));
    obs.push_back(0.5);  // command
    obs.push_back(0.0);  // command
    obs.push_back(0.0);  // command
    ioLcmInter_->get_status(obs);
    for (int i = 0; i < 12; i++) {
      obs.push_back(action_[i]);
    }
    obs.push_back(0.0);  // ang_vel
    obs.push_back(0.0);
    obs.push_back(0.0);
    obs.push_back(0.0);  // euler_xyz
    obs.push_back(0.0);
    obs.push_back(0.0);

    //
    if (obs_history_.size() == 15) {
      obs_history_.pop_front();  // 移除最旧的元素
    }
    obs_history_.push_back(obs);  // 添加新元素
}
