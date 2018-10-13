#ifndef PID_AGGREGATOR_H
#define PID_AGGREGATOR_H

#include <pid_control/pid_wrapper.h>
#include <au_core/DynamicsState.h>
#include <au_core/MCBaseSpeed.h>
#include <au_core/MCDiff.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace pid_control {

class PidAggregator {
 public:
  PidAggregator();
  ~PidAggregator();

  void start();
  void stop();

 protected:
  void readParams();
  void dispParams();

  bool isQuaternionValid(geometry_msgs::Quaternion q);

  void targetCb(const au_core::DynamicsState::ConstPtr& target);
  void stateCb(const au_core::DynamicsState::ConstPtr& state);

  void motorPub(const ros::TimerEvent& event);
  au_core::MCBaseSpeed genBaseSpeedMsg(float speed);
  au_core::MCDiff genDiffSpeedMsg(float speed);

  // ros node
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subs/pubs
  ros::Subscriber state_sub_;
  ros::Subscriber target_sub_;
  ros::Publisher error_pub_;
  ros::Publisher hor_base_pub_;
  ros::Publisher ver_base_pub_;
  ros::Publisher str_base_pub_;
  ros::Publisher hor_diff_pub_;
  ros::Publisher ver_diff_pub_;
  ros::Publisher str_diff_pub_;

  // timer
  ros::Timer motor_pub_timer_;

  // params
  int motor_rate_;  // loop rate in Hz
  std::string target_topic_;
  std::string error_topic_;
  std::string state_topic_;
  std::string pitch_ns_;
  std::string roll_ns_;
  std::string yaw_ns_;
  std::string depth_ns_;
  std::string ver_base_topic_;
  std::string hor_base_topic_;
  std::string str_base_topic_;
  std::string ver_diff_topic_;
  std::string hor_diff_topic_;
  std::string str_diff_topic_;
  double motor_max_;
  double depth_max_;
  bool normalized_;
  bool enabled_at_startup_;

  // target state
  double target_depth_;
  Eigen::Quaterniond target_orientation_;
  geometry_msgs::Vector3 target_linear_velocity_;
  geometry_msgs::Vector3 target_angular_velocity_;
  bool target_set_;

  std::unique_ptr<PidWrapper> pitchController_;
  std::unique_ptr<PidWrapper> rollController_;
  std::unique_ptr<PidWrapper> yawController_;
  std::unique_ptr<PidWrapper> depthController_;
};

}  // namespace pid_control

#endif  // PID_AGGREGATOR_H
