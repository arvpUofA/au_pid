#include <pid_control/pid_aggregator.h>
#include <au_core/math_util.h>
#include <eigen_conversions/eigen_msg.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>

using namespace pid_control;

PidAggregator::PidAggregator()
    : nh_(),
      private_nh_("~"),
      motor_rate_(10),
      motor_max_(80),
      depth_max_(10),
      target_set_(false),
      normalized_(false),
      enabled_at_startup_(false),
      target_topic_("/dynamics/target"),
      error_topic_("/dynamics/error"),
      state_topic_("/dynamics/state"),
      pitch_ns_("/pitch"),
      roll_ns_("/roll"),
      yaw_ns_("/yaw"),
      depth_ns_("/depth"),
      hor_base_topic_("/motor/hor/baseSpeed"),
      ver_base_topic_("/motor/ver/baseSpeed"),
      str_base_topic_("/motor/str/baseSpeed"),
      hor_diff_topic_("/motor/hor/diff"),
      ver_diff_topic_("/motor/ver/diff"),
      str_diff_topic_("/motor/str/diff") {
  readParams();
  dispParams();

  // init subscribers and publishers
  state_sub_ = nh_.subscribe<au_core::DynamicsState>(
      state_topic_, 1, &PidAggregator::stateCb, this);
  target_sub_ = nh_.subscribe<au_core::DynamicsState>(
      target_topic_, 1, &PidAggregator::targetCb, this);
  error_pub_ = nh_.advertise<au_core::DynamicsState>(error_topic_, 1);
  hor_base_pub_ = nh_.advertise<au_core::MCBaseSpeed>(hor_base_topic_, 1);
  ver_base_pub_ = nh_.advertise<au_core::MCBaseSpeed>(ver_base_topic_, 1);
  str_base_pub_ = nh_.advertise<au_core::MCBaseSpeed>(str_base_topic_, 1);
  hor_diff_pub_ = nh_.advertise<au_core::MCDiff>(hor_diff_topic_, 1);
  ver_diff_pub_ = nh_.advertise<au_core::MCDiff>(ver_diff_topic_, 1);
  str_diff_pub_ = nh_.advertise<au_core::MCDiff>(str_diff_topic_, 1);

  pitchController_ = PidWrapper::makeWrapper(pitch_ns_);
  rollController_ = PidWrapper::makeWrapper(roll_ns_);
  yawController_ = PidWrapper::makeWrapper(yaw_ns_);
  depthController_ = PidWrapper::makeWrapper(depth_ns_);

  motor_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / motor_rate_),
                                     &PidAggregator::motorPub, this);

  // wait for pid nodes to startup
  ros::Duration(5.0).sleep();

  if (enabled_at_startup_) {
    start();
  } else {
    stop();
  }
}

void PidAggregator::targetCb(const au_core::DynamicsState::ConstPtr& target) {
  if (!std::isnan(target->pose.position.z)) {
    target_depth_ = target->pose.position.z;
    target_depth_ = au_core::limitVariable(target_depth_, 0, depth_max_);
    depthController_->setTarget(
        normalized_
            ? au_core::normalizeVariable(target_depth_, 0, depth_max_, 0, 100)
            : target_depth_);
    ROS_WARN_COND(target_depth_ == depth_max_, "Setting max depth.");
  } else {
    ROS_WARN("Invalid depth target.");
  }

  if (isQuaternionValid(target->pose.orientation)) {
    tf::quaternionMsgToEigen(target->pose.orientation, target_orientation_);
    Eigen::Vector3d rpy = au_core::quatTorpy(target_orientation_);

    rollController_->setTarget(
        normalized_ ? au_core::normalizeVariable(rpy[0], -M_PI, M_PI, -100, 100)
                    : rpy[0]);
    pitchController_->setTarget(
        normalized_ ? au_core::normalizeVariable(rpy[1], -M_PI, M_PI, -100, 100)
                    : rpy[1]);
    yawController_->setTarget(
        normalized_ ? au_core::normalizeVariable(rpy[2], -M_PI, M_PI, -100, 100)
                    : rpy[2]);

    ROS_WARN_COND(rpy[0] != 0,
                  "Non-zero roll (%.3f rad) can make system unstable.", rpy[0]);
    ROS_WARN_COND(rpy[1] != 0,
                  "Non-zero pitch (%.3f rad) can make system unstable.",
                  rpy[1]);
  } else {
    ROS_WARN("Invalid orientation target.");
  }

  // set linear speed
  if (!std::isnan(target->velocity.linear.x)) {
    target_linear_velocity_.x = au_core::limitVariable(
        target->velocity.linear.x, -motor_max_, motor_max_);
  }

  if (!std::isnan(target->velocity.linear.y)) {
    target_linear_velocity_.y = au_core::limitVariable(
        target->velocity.linear.y, -motor_max_, motor_max_);
  }

  if (!std::isnan(target->velocity.linear.z)) {
    target_linear_velocity_.z = au_core::limitVariable(
        target->velocity.linear.z, -motor_max_, motor_max_);
  }

  // set angular speed
  if (!std::isnan(target->velocity.angular.x)) {
    target_angular_velocity_.x = au_core::limitVariable(
        target->velocity.angular.x, -motor_max_, motor_max_);
  }
  if (!std::isnan(target->velocity.angular.y)) {
    target_angular_velocity_.y = au_core::limitVariable(
        target->velocity.angular.y, -motor_max_, motor_max_);
  }
  if (!std::isnan(target->velocity.angular.z)) {
    target_angular_velocity_.z = au_core::limitVariable(
        target->velocity.angular.z, -motor_max_, motor_max_);
  }

  ROS_WARN_COND(target->pose.position.x != 0 || target->pose.position.y != 0,
                "Cannot set position x & y.");

  if (not target_set_) {
    target_set_ = true;
  }
}

void PidAggregator::stateCb(const au_core::DynamicsState::ConstPtr& state) {
  Eigen::Quaterniond qState;
  au_core::DynamicsState error_;
  if (isQuaternionValid(state->pose.orientation)) {
    tf::quaternionMsgToEigen(state->pose.orientation, qState);
    Eigen::Vector3d rpy = au_core::quatTorpy(qState);

    rollController_->setState(
        normalized_ ? au_core::normalizeVariable(rpy[0], -M_PI, M_PI, -100, 100)
                    : rpy[0]);
    pitchController_->setState(
        normalized_ ? au_core::normalizeVariable(rpy[1], -M_PI, M_PI, -100, 100)
                    : rpy[1]);
    yawController_->setState(
        normalized_ ? au_core::normalizeVariable(rpy[2], -M_PI, M_PI, -100, 100)
                    : rpy[2]);

    Eigen::Quaterniond qError = au_core::quatDiff(qState, target_orientation_);
    tf::quaternionEigenToMsg(qError, error_.pose.orientation);
  }

  if (!std::isnan(state->pose.position.z)) {
    depthController_->setState(
        normalized_ ? au_core::normalizeVariable(state->pose.position.z, 0,
                                                 depth_max_, 0, 100)
                    : state->pose.position.z);
    error_.pose.position.z = target_depth_ - state->pose.position.z;
  }

  // send error
  if (target_set_) {
    error_pub_.publish(error_);
  }
}

void PidAggregator::motorPub(const ros::TimerEvent& event) {
  hor_base_pub_.publish(
      genBaseSpeedMsg(static_cast<float>(target_linear_velocity_.x)));
  str_base_pub_.publish(
      genBaseSpeedMsg(static_cast<float>(target_linear_velocity_.y)));
  if (!depthController_->isEnabled()) {
    ver_base_pub_.publish(
        genBaseSpeedMsg(static_cast<float>(target_linear_velocity_.z)));
  } else {
    if (target_linear_velocity_.z != 0) {
      ROS_WARN_THROTTLE(
          30, "Cannot set vertical base speed with depth controller running");
    }
  }
  if (!rollController_->isEnabled()) {
    str_diff_pub_.publish(
        genDiffSpeedMsg(static_cast<float>(target_angular_velocity_.x)));
  } else {
    if (target_angular_velocity_.x != 0) {
      ROS_WARN_THROTTLE(
          30, "Cannot set x angular speed with roll controller running");
    }
  }
  if (!pitchController_->isEnabled()) {
    ver_diff_pub_.publish(
        genDiffSpeedMsg(static_cast<float>(target_angular_velocity_.y)));
  } else {
    if (target_angular_velocity_.y != 0) {
      ROS_WARN_THROTTLE(
          30, "Cannot set y angular speed with pitch controller running");
    }
  }
  if (!yawController_->isEnabled()) {
    hor_diff_pub_.publish(
        genDiffSpeedMsg(static_cast<float>(target_angular_velocity_.z)));
  } else {
    if (target_angular_velocity_.z != 0) {
      ROS_WARN_THROTTLE(
          30, "Cannot set z angular speed with yaw controller running");
    }
  }
}

au_core::MCBaseSpeed PidAggregator::genBaseSpeedMsg(float speed) {
  au_core::MCBaseSpeed baseSpeed;
  baseSpeed.baseSpeed = speed;
  baseSpeed.header.stamp = ros::Time::now();
  return baseSpeed;
}

au_core::MCDiff PidAggregator::genDiffSpeedMsg(float speed) {
  au_core::MCDiff diffSpeed;
  diffSpeed.differential = speed;
  diffSpeed.header.stamp = ros::Time::now();
  return diffSpeed;
}

void PidAggregator::start() {
  pitchController_->start();
  rollController_->start();
  yawController_->start();
  depthController_->start();
}

void PidAggregator::stop() {
  pitchController_->stop();
  rollController_->stop();
  yawController_->stop();
  depthController_->stop();
  hor_base_pub_.publish(genBaseSpeedMsg(0));
  str_base_pub_.publish(genBaseSpeedMsg(0));
  ver_base_pub_.publish(genBaseSpeedMsg(0));
}

void PidAggregator::readParams() {
  private_nh_.getParam("target_topic", target_topic_);
  private_nh_.getParam("error_topic", error_topic_);
  private_nh_.getParam("state_topic", state_topic_);
  private_nh_.getParam("motor_rate", motor_rate_);
  private_nh_.getParam("pitch_ns", pitch_ns_);
  private_nh_.getParam("roll_ns", roll_ns_);
  private_nh_.getParam("yaw_ns", yaw_ns_);
  private_nh_.getParam("depth_ns", depth_ns_);
  private_nh_.getParam("ver_base_topic", ver_base_topic_);
  private_nh_.getParam("hor_base_topic", hor_base_topic_);
  private_nh_.getParam("str_base_topic", str_base_topic_);
  private_nh_.getParam("ver_diff_topic", ver_diff_topic_);
  private_nh_.getParam("hor_diff_topic", hor_diff_topic_);
  private_nh_.getParam("str_diff_topic", str_diff_topic_);
  private_nh_.getParam("motor_max", motor_max_);
  private_nh_.getParam("depth_max", depth_max_);
  private_nh_.getParam("normalize", normalized_);
  private_nh_.getParam("enable_at_startup", enabled_at_startup_);
}

void PidAggregator::dispParams() {
  std::cout << "PID AGGREGATION PARAMETERS" << std::endl;
  std::cout << "-----------------------------------------" << std::endl;
  std::cout << "motor max: " << motor_max_ << std::endl;
  std::cout << "motor pub rate: " << motor_rate_ << " Hz" << std::endl;
  std::cout << "depth max: " << depth_max_ << " m" << std::endl;
  std::cout << "normalize: " << normalized_ << std::endl;
  std::cout << "target topic: " << target_topic_ << std::endl;
  std::cout << "error topic: " << error_topic_ << std::endl;
  std::cout << "state topic: " << state_topic_ << std::endl;
  std::cout << "ver_base topic: " << ver_base_topic_ << std::endl;
  std::cout << "hor_base topic: " << hor_base_topic_ << std::endl;
  std::cout << "str_base topic: " << str_base_topic_ << std::endl;
  std::cout << "ver_diff topic: " << ver_diff_topic_ << std::endl;
  std::cout << "hor_diff topic: " << hor_diff_topic_ << std::endl;
  std::cout << "str_diff topic: " << str_diff_topic_ << std::endl;
  std::cout << "depth_ns: " << depth_ns_ << std::endl;
  std::cout << "roll_ns: " << roll_ns_ << std::endl;
  std::cout << "pitch_ns: " << pitch_ns_ << std::endl;
  std::cout << "yaw_ns: " << yaw_ns_ << std::endl;
  std::cout << "enabled: " << enabled_at_startup_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl
            << std::endl;
}

bool PidAggregator::isQuaternionValid(geometry_msgs::Quaternion q) {
  if (!std::isnan(q.x) and !std::isnan(q.y) and !std::isnan(q.z) and
      !std::isnan(q.w)) {
    const double epsilon = 0.01;
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return std::abs(1 - norm) <= epsilon;
  }
  return false;
}

PidAggregator::~PidAggregator() = default;
