#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <condition_variable>
#include <thread>

#include "api.h"

struct JointCalibController_DLLAPI JointCalibController : public mc_control::fsm::Controller
{
  JointCalibController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  ~JointCalibController();

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  mc_rtc::Configuration config_;

  std::map<std::string, double> msg;
  std::string datastoreName_ = "PolicyPredictions";
  bool bypassQP_ = true;

  // thread NN model execution
  std::thread policy_th_;
  bool policy_th_exit_ = false;
  std::function<void()> * policy_work_ = nullptr;
  std::condition_variable cv;
  std::mutex policy_mutex_;
};
