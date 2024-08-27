/*
 * @Author: aoi
 * @Date: 2024-08-26 20:23:04
 * @LastEditors: aoi
 * @LastEditTime: 2024-08-27 11:11:07
 * @Description:
 * Copyright (c) Air by aoi, All Rights Reserved.
 */
#include "../include/lcm/lcm02_IO.h"
// #include "../include/R.h"
#include <unistd.h>

#include <csignal>
#include <iostream>

// 接受Fb，发送Des

IOLcm::IOLcm(const std::string& robot_name)
    : _running(true),
      _lcm("udpm://239.255.76.67:7667?ttl=1"),
      _robot_name(robot_name) {
  std::cout << "The Mpc interface is initialized" << std::endl;

  if (!_lcm.good()) {
    std::cerr << "Error: lcm is not good" << std::endl;
  }

  initRecv();
  usleep(2000);
  _lcmThread = std::thread(&IOLcm::lcmHandler, this);
}

IOLcm::~IOLcm() {
  _running = false;
  if (_lcmThread.joinable()) {
    _lcmThread.join();
  }
  std::cout << "lcmThread joined" << std::endl;
}

void IOLcm::sendOutput(const std::vector<double>& action) {
  constexpr int defaultPositions[19] = {
      90192, 5292,  -44683,  199904,  100000, -100000, -90193,
      -5293, 44682, -199905, -100000, 100000, 0,       0,
      0,     0,     0,       0,       0};
  constexpr double defaultValue = 2.0;

  for (int i = 0; i < 12; ++i) {
    _lcm_Pub_Des.qDes[i] = action[i];
    _lcm_Pub_Des.dqDes[i] = defaultValue;
    _lcm_Pub_Des.tauDes[i] = defaultValue;
    _lcm_Pub_Des.Kp[i] = defaultValue;
    _lcm_Pub_Des.Kd[i] = defaultValue;
  }

  _lcm.publish("/" + _robot_name + "/RealDes", &_lcm_Pub_Des);
  std::cout << " _lcm_Pub_Des.qDes[1] = " << _lcm_Pub_Des.qDes[1] << std::endl;

  _if_recv = false;
}

void IOLcm::initRecv() {
  _lcm.subscribe("/" + _robot_name + "/OutputFb", &IOLcm::InputCallback, this);
}

void IOLcm::InputCallback(const lcm::ReceiveBuffer* rbuf,
                          const std::string& chan, const RealFbLcm* msg) {
  for (int i = 0; i < 12; i++) {
    _lcm_Rec_Fb.qFb[i] = msg->qFb[i];
    _lcm_Rec_Fb.dqFb[i] = msg->dqFb[i];
    _lcm_Rec_Fb.tauFb[i] = msg->tauFb[i];
  }

  for (int i = 0; i < 4; i++) {
    _lcm_Rec_Fb.imu_quat[i] = msg->imu_quat[i];
  }

  for (int i = 0; i < 3; i++) {
    _lcm_Rec_Fb.imu_gyro[i] = msg->imu_gyro[i];
    _lcm_Rec_Fb.imu_acc[i] = msg->imu_acc[i];
  }

  _if_recv = true;
  std::cout << "InputCallback" << std::endl;
  std::cout << "02Fb qFb" << _lcm_Rec_Fb.qFb[1] << std::endl;
}

void IOLcm::get_status(std::vector<double>& status) {
  for (int i = 0; i < 12; i++) {
    status.push_back(_lcm_Rec_Fb.qFb[i]);
  }

  for (int i = 0; i < 12; i++) {
    status.push_back(_lcm_Rec_Fb.dqFb[i]);
  }
}

void IOLcm::lcmHandler() {
  while (_running) {
    _lcm.handleTimeout(1000);
  }
}
