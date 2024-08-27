/*
 * @Author: aoi
 * @Date: 2024-07-19 10:34:32
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-27 10:46:05
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#pragma once

namespace humanoid {

class HumanoidControlParam {
 public:
  // 禁止拷贝构造和赋值操作
  HumanoidControlParam(const HumanoidControlParam&) = delete;
  HumanoidControlParam& operator=(const HumanoidControlParam&) = delete;

  static HumanoidControlParam& get_instance() {
    static HumanoidControlParam instance;
    return instance;
  }

  double gait_cycle_time = 640.0;  // ms
  int sliding_windows_size = 15;
  int single_obs_size = 47;
  int sum_obs_size = sliding_windows_size * single_obs_size;
  int action_size = 12;
  std::string model_path =
      "/home/fudanrobotuser/humanoid_ws/src/"
      "humanoid_control/model/model_rl.pt";

 private:
  // 私有化构造函数以防止外部实例化
  HumanoidControlParam() {}
  ~HumanoidControlParam() {}
};

}  // namespace humanoid