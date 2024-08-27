/*
 * @Author: aoi
 * @Date: 2024-08-26 17:57:02
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-27 11:23:57
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#include "../include/humanoid_control_interface.h"

#include <cmath>
#include <iostream>

#include "../include/humanoid_control_param.h"

using namespace std::chrono_literals;
using namespace humanoid;
HumanoidControlInterface::HumanoidControlInterface(
    const rclcpp::NodeOptions& options)
    : Node("humanoid_control_node", options), ioLcmInter_("fudan_humanoid") {}

bool HumanoidControlInterface::init() {  //
  HumanoidControlParam& param = HumanoidControlParam::get_instance();
  //
  obs_history_.resize(0);
  for (int i = 0; i < param.sliding_windows_size; ++i) {
    std::vector<double> new_vector(param.single_obs_size);
    obs_history_.push_back(new_vector);
  }
  //
  action_.resize(0);
  for (int i = 0; i < param.action_size; i++) {
    action_.push_back(0.0);
  }
  //
  try {
    // 使用 std::make_shared 包装返回的 torch::jit::Module
    model_rl_ = std::make_shared<torch::jit::Module>(
        torch::jit::load(param.model_path));
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

  return true;
}

void HumanoidControlInterface::inference() {
  std::lock_guard<std::mutex> lock(mtx_inference_);
  HumanoidControlParam& param = HumanoidControlParam::get_instance();

  observation();

  std::vector<float> input_data;
  input_data.reserve(param.sum_obs_size);  // 提前分配空间
  for (const auto& vector : obs_history_) {
    input_data.insert(input_data.end(), vector.begin(),
                      vector.end());  // 批量插入
  }

  torch::Tensor input_tensor =
      torch::from_blob(input_data.data(), {1, param.sum_obs_size});

  // 执行推理
  auto action = model_rl_->forward({input_tensor}).toTensor();
  for (int i = 0; i < param.action_size; i++) {
    action_[i] = action.index({0, i}).item<double>();
  }

  std::cout << action_ << std::endl;
}

void HumanoidControlInterface::control() {
  std::lock_guard<std::mutex> lock(mtx_control_);

  // 发送action_ pd 调节
  ioLcmInter_.sendOutput(action_);
}

void HumanoidControlInterface::observation() {
  std::lock_guard<std::mutex> lock(mtx_observation_);
  HumanoidControlParam& param = HumanoidControlParam::get_instance();
  std::vector<double> obs;  //
  obs.reserve(param.single_obs_size);  
  // 记录结束时间点
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time_).count();

  obs.push_back(sin(2.0 * M_PI * duration / param.gait_cycle_time));
  obs.push_back(cos(2.0 * M_PI * duration / param.gait_cycle_time));
  obs.insert(obs.end(), {0.5, 0.0, 0.0});  // command
  ioLcmInter_.get_status(obs);
  obs.insert(obs.end(), action_.begin(), action_.end());
  obs.insert(obs.end(), {0.0, 0.0, 0.0});  // ang_vel
  obs.insert(obs.end(), {0.0, 0.0, 0.0});  // euler_xyz

  //
  if (obs_history_.size() == param.sliding_windows_size) {
    obs_history_.pop_front();  // 移除最旧的元素
  }
  obs_history_.push_back(obs);  // 添加新元素
}
