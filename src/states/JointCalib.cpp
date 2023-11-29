#include "JointCalib.h"

void JointCalib::configure(const mc_rtc::Configuration & config)
{
  // load configuration
  config_.load(config);
}

void JointCalib::verifyServoGains(mc_control::fsm::Controller & ctl)
{
  const auto & rjo = ctl.robot().module().ref_joint_order();
  for(unsigned int i = 0; i < active_motors_.size(); i++)
  {
    auto rjo_it = std::find(rjo.begin(), rjo.end(), active_motors_[i]);
    if(rjo_it == rjo.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Motor name \"{}\" not found in ref_joint_order: {}.",
                                                       name(), active_motors_[i]);
    }

    bool get_servo_success = false;
    double kp, kd;
    if(ctl.datastore().has(ctl.robot().name() + "::GetPDGainsByName"))
    {
      get_servo_success = ctl.datastore().call<bool, const std::string &, double &, double &>(
          ctl.robot().name() + "::GetPDGainsByName", active_motors_[i], kp, kd);
    }
    if(!get_servo_success)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Could not get PD gains through datastore call!!!", name());
    }

    auto check_and_throw = [&](const std::string & s, double exp, double act) {
      if(std::abs(exp - act) > 0.1)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[{}] Unexpected servo {} gain for joint {}!!! Expected={}. Is={}", name(), s, active_motors_[i], exp, act);
      }
    };
    check_and_throw("kp", motor_kps_[i], kp);
    check_and_throw("kd", motor_kds_[i], kd);
  }
}

void JointCalib::start(mc_control::fsm::Controller & ctl)
{
  // load robot-specific configuration
  const auto& robot_config_ = config_(ctl.robot().name());
  robot_config_("activeMotors", active_motors_);
  robot_config_("motorPGains", motor_kps_);
  robot_config_("motorDGains", motor_kds_);
  robot_config_("qLimitLower", qlim_lower_);
  robot_config_("qLimitUpper", qlim_upper_);
  robot_config_("periodSeconds", cycle_periods_s_);
  robot_config_("numCycles", num_cycles_);

  // simple sanity check to make sure qlimits don't exceed module defined limits
  unsigned int num_motors = active_motors_.size();
  if((qlim_lower_.size() == num_motors) && (qlim_upper_.size() == num_motors))
  {
    for(unsigned int i = 0; i < active_motors_.size(); i++)
    {
      int mbc_id = ctl.robot().jointIndexByName(active_motors_[i]);
      double ql = ctl.robot().ql()[mbc_id][0] * 180 / PI;
      double qu = ctl.robot().qu()[mbc_id][0] * 180 / PI;
      if(qlim_lower_[i] < ql)
      {
	qlim_lower_[i] = ql;
      }
      if(qlim_upper_[i] > qu)
      {
	qlim_upper_[i] = qu;
      }
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] qlim_lower_ (size={}) and qlim_upper_ (size={}) should be of size={}.", name(), qlim_lower_.size(), qlim_upper_.size(), num_motors);
  }

  // Change the limits
  for(const auto & jn : active_motors_)
  {
    ctl.robot().ql()[ctl.robot().jointIndexByName(jn)][0] = -std::numeric_limits<double>::infinity();
    ctl.robot().qu()[ctl.robot().jointIndexByName(jn)][0] = std::numeric_limits<double>::infinity();
  }
  // Re-create the kinematic constraint with those limits
  std::array<double, 3> damper{0.1, 0.01, 0.5};
  ctl.kinematicsConstraint.reset(new mc_solver::KinematicsConstraint(ctl.robots(), 0, ctl.timeStep, damper, 0.5));

  // verify if expected PD gains are loaded
  verifyServoGains(ctl);

  if(!ctl.datastore().has(datastoreName_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}] Datastore entry {} for storing joint targets does not exist.", name(), datastoreName_);
  }

  // zero the command torques
  command_torques = std::vector<double>(num_motors, 0);

  addLogEntries(ctl);
  createGUI(ctl);
  mc_rtc::log::success("[{}] Started successfully.", name());
}

bool JointCalib::run(mc_control::fsm::Controller & ctl)
{

  // check if robot is in the air
  Eigen::Vector3d rfoot_frc = ctl.realRobot().forceSensor(sensor_names[0]).force();
  Eigen::Vector3d lfoot_frc = ctl.realRobot().forceSensor(sensor_names[1]).force();
  bool robot_on_ground = (rfoot_frc[2] > 100 && lfoot_frc[2] > 100);
  if(robot_on_ground)
  {
    mc_rtc::log::error("[{}] Robot is on the ground. Emergency trigerred!", name());
    output("OK");
    return false;
  }

  if(active_)
  {
    std::vector<double> target;
    // compute the target joint position
    for(unsigned int i = 0; i < active_motors_.size(); i++)
    {
      int cycle_period = cycle_periods_s_[cycleCounter_]/ctl.timeStep;
      double range = (qlim_upper_[i] - qlim_lower_[i]);
      double mid_pt = qlim_lower_[i] + range/2;
      double t = sin(2 * PI * iterCounter_ / cycle_period);
      target.push_back((mid_pt + t*range/2) * PI / 180);
    }

    // set posture task targets
    unsigned int target_idx_ = 0;
    for(auto i : active_motors_)
    {
      // log the computed torques
      int idx = ctl.robot().jointIndexByName(i);
      double q = ctl.realRobot().mbc().q[idx][0];
      double qd = ctl.realRobot().mbc().alpha[idx][0];
      double qerr = (target[target_idx_] - q);
      double qderr = (0 - qd);
      double tau_out = motor_kps_[target_idx_] * (qerr) + motor_kds_[target_idx_] * (qderr);
      command_torques[target_idx_] = tau_out;

      // check for joint position limit safety
      double llimit = qlim_lower_[target_idx_] * PI / 180;
      double ulimit = qlim_upper_[target_idx_] * PI / 180;
      if((q < llimit) || (q > ulimit))
      {
        //active_ = false;
        mc_rtc::log::warning("[{}] Joint {} position exceeds the defined limits!! "
                             "(q={} deg, llimit={} deg, ulimit={} deg)",
                             name(), i, q * 180 / PI, llimit * 180 / PI, ulimit * 180 / PI);
        //break;
      }

      control_msg[i] = target[target_idx_];
      target_idx_++;
    }

    ctl.datastore().assign(datastoreName_, control_msg);
  }

  // move to next cycle after num_cycles_
  if((iterCounter_*ctl.timeStep) >= (num_cycles_*cycle_periods_s_[cycleCounter_]))
  {
    cycleCounter_++;
    iterCounter_ = 0;
    mc_rtc::log::success("[{}] Completed cycle {}.", name(), cycleCounter_);
  }

  // deactivate after all cycles
  if(cycleCounter_ >= cycle_periods_s_.size())
  {
    active_ = false;
    mc_rtc::log::success("[{}] Completed (iterations = {}).", name(), iterCounter_);
    output("OK");
    return true;
  }

  iterCounter_++;
  output("OK");
  return false;
}

void JointCalib::addLogEntries(mc_control::fsm::Controller & ctl)
{
  ctl.logger().addLogEntry(name() + "_tauRef", [this]() { return command_torques; });
}

void JointCalib::teardown(mc_control::fsm::Controller & ctl)
{
  active_ = false;
  ctl.logger().removeLogEntry(name() + "_tauRef");
  ctl.gui()->removeCategory({"RL", name()});
}

void JointCalib::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
  gui.addElement({"RL", name()}, mc_rtc::gui::Label("Active", [this]() { return active_; }),
                 mc_rtc::gui::Checkbox(
                     "Activated", [this]() { return active_; }, [this]() { active_ = !active_; }));
}

EXPORT_SINGLE_STATE("JointCalib", JointCalib)
