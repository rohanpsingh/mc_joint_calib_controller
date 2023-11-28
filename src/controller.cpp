#include "controller.h"

JointCalibController::JointCalibController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)

: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);
  const auto& robot_config_ = config_(robot().name());

  datastore().make<std::map<std::string, double>>(datastoreName_);

  // create new thread for executing the policies
  policy_th_ = std::thread([this]() {
    while(!policy_th_exit_)
    {
      std::unique_lock lk(policy_mutex_);
      cv.wait(lk, [this] { return (policy_work_ != nullptr); });

      (*policy_work_)();
      policy_work_ = nullptr;

      lk.unlock();
      cv.notify_one();
    }
  });

  // create gui
  gui()->addElement({"RL"}, mc_rtc::gui::Checkbox(
                                "bypassQP?", [this]() { return bypassQP_; }, [this]() { bypassQP_ = !bypassQP_; }));
  gui()->addElement({"RL"}, mc_rtc::gui::Label("Policy thread id:", [this]() {
                      std::stringstream ss;
                      ss << policy_th_.get_id();
                      return ss.str();
                    }));

  mc_rtc::log::success("JointCalibController init done");
}

JointCalibController::~JointCalibController()
{
  std::function<void()> exit = []() {};

  {
    std::lock_guard lk(policy_mutex_);
    policy_th_exit_ = true;
    policy_work_ = &exit;
  }
  cv.notify_one();
  policy_th_.join();
}

bool JointCalibController::run()
{
  if(mc_control::fsm::Controller::run())
  {
    if(!datastore().has(datastoreName_))
    {
      return true;
    }
    msg = datastore().get<std::map<std::string, double>>(datastoreName_);
    if(bypassQP_)
    {
      for(auto i : msg)
      {
        auto jIndex = robot().jointIndexByName(i.first);
        robot().mbc().q[jIndex] = std::vector<double>{i.second};
        robot().mbc().alpha[jIndex] = std::vector<double>{0};
      }
    }
    else
    {
      for(auto i : msg)
      {
        getPostureTask(robot().name())->target({{i.first, std::vector<double>{i.second}}});
      }
    }
    return true;
  }
  return false;
}

void JointCalibController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  datastore().make_call("ExecuteActorCritic", [this](std::function<void()> & work) {
    {
      std::lock_guard lk(policy_mutex_);
      policy_work_ = &work;
    }
    cv.notify_one();

    {
      std::unique_lock lk(policy_mutex_);
      cv.wait(lk, [this] { return (policy_work_ == nullptr); });
    }
  });
}
