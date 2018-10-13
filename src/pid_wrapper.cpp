#include <pid_control/pid_wrapper.h>

using namespace pid_control;

PidWrapper::PidWrapper(std::string ns) : nh_(), ns_(ns), enabled_(false) {
  // init publishers and subscribers
  target_pub_ = nh_.advertise<std_msgs::Float64>(ns_ + "/setpoint", 1);
  state_pub_ = nh_.advertise<std_msgs::Float64>(ns_ + "/state", 1);
  enable_pub_ = nh_.advertise<std_msgs::Bool>(ns_ + "/pid_enable", 1);
  enable_sub_ = nh_.subscribe<std_msgs::Bool>(ns_ + "/pid_enable", 1,
                                              &PidWrapper::enableCb, this);
}

void PidWrapper::enableCb(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != enabled_)  // state change
  {
    ROS_INFO("%s: %s", ns_.c_str(), msg->data ? "enabled" : "disabled");
  }
  enabled_ = msg->data;
}

std::unique_ptr<PidWrapper> PidWrapper::makeWrapper(std::string ns) {
  return std::make_unique<PidWrapper>(ns);
}

void PidWrapper::start() {
  std_msgs::Bool msg;
  msg.data = static_cast<u_char>(true);
  enable_pub_.publish(msg);
}

void PidWrapper::stop() {
  std_msgs::Bool msg;
  msg.data = static_cast<u_char>(false);
  enable_pub_.publish(msg);
}

bool PidWrapper::isEnabled() { return enabled_; }

void PidWrapper::setTarget(double target) {
  std_msgs::Float64 msg;
  msg.data = target;
  target_pub_.publish(msg);

  if (!enabled_) {
    ROS_WARN("%s: target sent but controller is disabled.", ns_.c_str());
  }
}

void PidWrapper::setState(double state) {
  std_msgs::Float64 msg;
  msg.data = state;
  state_pub_.publish(msg);
}
