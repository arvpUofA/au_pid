#ifndef PID_WRAPPER_H
#define PID_WRAPPER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <memory>

namespace pid_control {

class PidWrapper {
 public:
  explicit PidWrapper(std::string ns);
  ~PidWrapper() = default;

  static std::unique_ptr<PidWrapper> makeWrapper(std::string ns);

  // manage enable/disable
  void start();
  void stop();
  bool isEnabled();

  // publish state/target
  void setTarget(double target);
  void setState(double state);

 protected:
  // callbacks
  void enableCb(const std_msgs::Bool::ConstPtr& msg);

  ros::NodeHandle nh_;

  // publishers and subscribers
  ros::Publisher target_pub_;
  ros::Publisher state_pub_;
  ros::Publisher enable_pub_;
  ros::Subscriber enable_sub_;

  // params
  std::string ns_;

  // state
  bool enabled_;
};

}  // namespace au_control

#endif  // PID_WRAPPER_H
