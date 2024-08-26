/*
 * @Author: aoi
 * @Date: 2024-08-26 17:57:02
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-26 19:44:21
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#include "../include/humanoid_control_interface.h"

#include <iostream>

using namespace std::chrono_literals;
using namespace humanoid;
HumanoidControlInterface::HumanoidControlInterface(
    const rclcpp::NodeOptions& options)
    : Node("humanoid_control_node", options) {}

bool HumanoidControlInterface::init() {  //

  // 初始化15个包含47个元素的vector，并将它们添加到obs_history_中
  for (int i = 0; i < 15; ++i) {
    // 创建一个新的std::vector<double>，大小为47，并初始化为0.0（这是默认行为）
    std::vector<double> new_vector(47);

    // 如果需要，你可以在这里设置一个特定的值给所有元素
    // 例如，将每个元素的值设置为i（注意：这通常不是一个好的选择，因为你会覆盖所有元素为相同的i值）
    // 但这里我们保持默认值0.0

    // 将这个新的vector添加到deque中
    obs_history_.push_back(new_vector);
  }

  try {
    // 使用 std::make_shared 包装返回的 torch::jit::Module
    model_rl_ = std::make_shared<torch::jit::Module>(torch::jit::load(
        "/home/imatrix/humanoid_ws/src/humanoid_control/model/model_rl.pt"));
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
  std::vector<float> input_data;
  for (auto& vector : obs_history_) {
    for (auto& element : vector) {
      input_data.push_back(element);
    }
  }
  torch::Tensor input_tensor = torch::from_blob(input_data.data(), {1, 705});

  // 执行推理
  action_ = model_rl_->forward({input_tensor}).toTensor();

  // 输出结果
  //   for (const auto& tensor : output) {
  //         std::cout << tensor << std::endl; //
  //         注意：这不会直接打印张量的内容，而是其类型和可能的内存地址
  //         //
  //         如果你需要打印张量的具体内容，你可能需要使用其他方法，比如遍历其元素
  //         // 示例：打印第一个张量的第一个元素
  //         std::cout << "First element of the first tensor: " <<
  //         tensor[0][0].item<double>() << std::endl;
  //     }
  std::cout << action_ << std::endl;
}

void HumanoidControlInterface::control() {
  std::lock_guard<std::mutex> lock(mtx_control_);

  static int count = 0;
  count++;

  if (count++ % 10 == 0) {
    // create obs
    std::vector<double> obs;
    add_obs(obs);
    count = 0;
  }
  // 发送action_
  send_command();
}

void HumanoidControlInterface::send_command() {}

void HumanoidControlInterface::add_obs(const std::vector<double>& obs) {
  if (obs_history_.size() == 15) {
    obs_history_.pop_front();  // 移除最旧的元素
  }
  obs_history_.push_back(obs);  // 添加新元素
}
