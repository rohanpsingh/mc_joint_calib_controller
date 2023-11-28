#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>

#define PI 3.14159265

struct JointCalib : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void verifyServoGains(mc_control::fsm::Controller & ctl);
  void createGUI(mc_control::fsm::Controller & ctl);
  void addLogEntries(mc_control::fsm::Controller & ctl);

private:
  mc_rtc::Configuration config_;

  /// @{ CONFIG
  /**
   * Name of joints to be moved.
   */
  std::vector<std::string> active_motors_;
  /**
   * Servo kP gain (used for verification).
   */
  std::vector<double> motor_kps_;
  /**
   * Servo kD gain (used for verification).
   */
  std::vector<double> motor_kds_;
  /**
   * Lower limit (in degrees) for joint position limits.
   */
  std::vector<double> qlim_lower_;
  /**
   * Upper limit (in degrees) for joint position limits.
   */
  std::vector<double> qlim_upper_;
  /**
   * Number of timesteps in each cycle (controls motion speed).
   */
  double cycle_period_s_ = 1.0;
  /**
   * Number of cycles before deactivating.
   */
  int num_cycles_ = 1;
  ///@}

  std::string datastoreName_ = "PolicyPredictions";
  std::map<std::string, double> control_msg;
  std::vector<std::string> sensor_names = {"RightFootForceSensor", "LeftFootForceSensor"};

  // Torque commanded by policy
  std::vector<double> command_torques;

  // Set 'true' to enable updating control message
  bool active_ = true;

  // Counter for run iterations
  int iterCounter_ = 0;
};
