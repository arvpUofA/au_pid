#include <eigen_conversions/eigen_msg.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>

#include <pid_control/pid_aggregator.h>
#include <au_core/math_util.h>

#define MESSAGE_TIMEOUT 6

namespace pid_control {
class PidAggregatorTest : public ::testing::Test {
 public:
  PidAggregatorTest()
      : nh_(),
        statePub_(nh_.advertise<au_core::DynamicsState>("/state", 1)),
        targetPub_(nh_.advertise<au_core::DynamicsState>("/target", 1)),
        errorSub_(nh_.subscribe("/error", 1, &PidAggregatorTest::dummyDynamicCb,
                                this)),
        depthStateSub_(nh_.subscribe("/depth/state", 1,
                                     &PidAggregatorTest::dummyFloatCb, this)),
        depthTargetSub_(nh_.subscribe("/depth/setpoint", 1,
                                      &PidAggregatorTest::dummyFloatCb, this)),
        rollStateSub_(nh_.subscribe("/roll/state", 1,
                                    &PidAggregatorTest::dummyFloatCb, this)),
        rollTargetSub_(nh_.subscribe("/roll/setpoint", 1,
                                     &PidAggregatorTest::dummyFloatCb, this)),
        pitchStateSub_(nh_.subscribe("/pitch/state", 1,
                                     &PidAggregatorTest::dummyFloatCb, this)),
        pitchTargetSub_(nh_.subscribe("/pitch/setpoint", 1,
                                      &PidAggregatorTest::dummyFloatCb, this)),
        yawStateSub_(nh_.subscribe("/yaw/state", 1,
                                   &PidAggregatorTest::dummyFloatCb, this)),
        yawTargetSub_(nh_.subscribe("/yaw/setpoint", 1,
                                    &PidAggregatorTest::dummyFloatCb, this)),
        horBasespeedSub_(nh_.subscribe(
            "/hor/base", 1, &PidAggregatorTest::dummyBasespeedCb, this)),
        verBasespeedSub_(nh_.subscribe(
            "/ver/base", 1, &PidAggregatorTest::dummyBasespeedCb, this)),
        strBasespeedSub_(nh_.subscribe(
            "/str/base", 1, &PidAggregatorTest::dummyBasespeedCb, this)) {}

  void SetUp() {
    while (!IsNodeReady()) {
      ros::spinOnce();
    }
  }

  void PublishState(geometry_msgs::Point position,
                    geometry_msgs::Quaternion orientation) {
    au_core::DynamicsState state;
    state.pose.position = position;
    state.pose.orientation = orientation;
    state.header.stamp = ros::Time::now();
    statePub_.publish(state);
  }

  void PublishState(geometry_msgs::Point position) {
    PublishState(position, geometry_msgs::Quaternion());
  }

  void PublishState(geometry_msgs::Quaternion quaternion) {
    PublishState(geometry_msgs::Point(), quaternion);
  }

  void PublishTarget(geometry_msgs::Point position,
                     geometry_msgs::Quaternion orientation,
                     geometry_msgs::Twist velocity) {
    au_core::DynamicsState target;
    target.pose.position = position;
    target.pose.orientation = orientation;
    target.velocity = velocity;
    target.header.stamp = ros::Time::now();
    targetPub_.publish(target);
  }

  void PublishTarget(geometry_msgs::Point position) {
    PublishTarget(position, geometry_msgs::Quaternion(),
                  geometry_msgs::Twist());
  }

  void PublishTarget(geometry_msgs::Quaternion orientation) {
    PublishTarget(geometry_msgs::Point(), orientation, geometry_msgs::Twist());
  }

  void PublishTarget(geometry_msgs::Twist velocity) {
    PublishTarget(geometry_msgs::Point(), geometry_msgs::Quaternion(),
                  velocity);
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForDepthState() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        depthStateSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForDepthTarget() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        depthTargetSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForRollState() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        rollStateSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForRollTarget() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        rollTargetSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForPitchState() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        pitchStateSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForPitchTarget() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        pitchTargetSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForYawState() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        yawStateSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const std_msgs::Float64> WaitForYawTarget() {
    return ros::topic::waitForMessage<std_msgs::Float64>(
        yawTargetSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const au_core::DynamicsState> WaitForError() {
    return ros::topic::waitForMessage<au_core::DynamicsState>(
        errorSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const au_core::MCBaseSpeed> WaitForHorBasespeed() {
    return ros::topic::waitForMessage<au_core::MCBaseSpeed>(
        horBasespeedSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const au_core::MCBaseSpeed> WaitForStrBasespeed() {
    return ros::topic::waitForMessage<au_core::MCBaseSpeed>(
        strBasespeedSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

  boost::shared_ptr<const au_core::MCBaseSpeed> WaitForVerBasespeed() {
    return ros::topic::waitForMessage<au_core::MCBaseSpeed>(
        verBasespeedSub_.getTopic(), ros::Duration(MESSAGE_TIMEOUT));
  }

 private:
  ros::NodeHandle nh_;
  ros::Publisher statePub_;
  ros::Publisher targetPub_;
  ros::Subscriber errorSub_;
  ros::Subscriber depthStateSub_;
  ros::Subscriber depthTargetSub_;
  ros::Subscriber rollStateSub_;
  ros::Subscriber rollTargetSub_;
  ros::Subscriber pitchStateSub_;
  ros::Subscriber pitchTargetSub_;
  ros::Subscriber yawStateSub_;
  ros::Subscriber yawTargetSub_;
  ros::Subscriber horBasespeedSub_;
  ros::Subscriber verBasespeedSub_;
  ros::Subscriber strBasespeedSub_;

  bool IsNodeReady() {
    return (statePub_.getNumSubscribers() > 0) &&
           (targetPub_.getNumSubscribers() > 0) &&
           (depthStateSub_.getNumPublishers() > 0) &&
           (depthTargetSub_.getNumPublishers() > 0) &&
           (rollStateSub_.getNumPublishers() > 0) &&
           (rollTargetSub_.getNumPublishers() > 0) &&
           (pitchTargetSub_.getNumPublishers() > 0) &&
           (pitchStateSub_.getNumPublishers() > 0) &&
           (yawTargetSub_.getNumPublishers() > 0) &&
           (yawStateSub_.getNumPublishers() > 0);
  }

  void dummyFloatCb(const std_msgs::Float64::ConstPtr msg) {}
  void dummyBasespeedCb(const au_core::MCBaseSpeed::ConstPtr msg) {}
  void dummyDynamicCb(const au_core::DynamicsState::ConstPtr msg) {}

};  // end of class

void EXPECT_QUATERNION_EQ(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
  EXPECT_NEAR(std::abs(q1.dot(q2)), 1, 0.01);
}

TEST_F(PidAggregatorTest, DepthState) {
  geometry_msgs::Point position;
  position.z = 3;
  PublishState(position);
  auto output = WaitForDepthState();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(position.z, output->data);
}

TEST_F(PidAggregatorTest, DepthTarget) {
  geometry_msgs::Point position;
  position.z = 3;
  PublishTarget(position);
  auto output = WaitForDepthTarget();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(position.z, output->data);
}

TEST_F(PidAggregatorTest, RollState) {
  double roll = 1.4;
  Eigen::Quaterniond q = au_core::rpyToQuat(Eigen::Vector3d(roll, 0.3, 1.2));
  geometry_msgs::Quaternion orientation;
  tf::quaternionEigenToMsg(q, orientation);
  PublishState(orientation);
  auto output = WaitForRollState();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(roll, output->data);
}

TEST_F(PidAggregatorTest, RollTarget) {
  double roll = 1.0;
  Eigen::Quaterniond q = au_core::rpyToQuat(Eigen::Vector3d(roll, 0.3, 1.2));
  geometry_msgs::Quaternion orientation;
  tf::quaternionEigenToMsg(q, orientation);
  PublishTarget(orientation);
  auto output = WaitForRollTarget();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(roll, output->data);
}

TEST_F(PidAggregatorTest, PitchState) {
  double pitch = 0.7;
  Eigen::Quaterniond q = au_core::rpyToQuat(Eigen::Vector3d(0.1, pitch, 1.2));
  geometry_msgs::Quaternion orientation;
  tf::quaternionEigenToMsg(q, orientation);
  PublishState(orientation);
  auto output = WaitForPitchState();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(pitch, output->data);
}

TEST_F(PidAggregatorTest, PitchTarget) {
  double pitch = 1.0;
  Eigen::Quaterniond q = au_core::rpyToQuat(Eigen::Vector3d(0.5, pitch, 1.2));
  geometry_msgs::Quaternion orientation;
  tf::quaternionEigenToMsg(q, orientation);
  PublishTarget(orientation);
  auto output = WaitForPitchTarget();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(pitch, output->data);
}

TEST_F(PidAggregatorTest, YawState) {
  double yaw = M_PI - 0.1;
  Eigen::Quaterniond q = au_core::rpyToQuat(Eigen::Vector3d(0.1, 0.2, yaw));
  geometry_msgs::Quaternion orientation;
  tf::quaternionEigenToMsg(q, orientation);
  PublishState(orientation);
  auto output = WaitForYawState();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(yaw, output->data);
}

TEST_F(PidAggregatorTest, YawTarget) {
  double yaw = 0.1 - M_PI;
  Eigen::Quaterniond q = au_core::rpyToQuat(Eigen::Vector3d(0.5, -0.2, yaw));
  geometry_msgs::Quaternion orientation;
  tf::quaternionEigenToMsg(q, orientation);
  PublishTarget(orientation);
  auto output = WaitForYawTarget();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(yaw, output->data);
}

TEST_F(PidAggregatorTest, LinearHorSpeed) {
  double baseSpeed = -69;
  geometry_msgs::Twist velocity;
  velocity.linear.x = baseSpeed;
  PublishTarget(velocity);
  ros::Duration(1).sleep();
  auto output = WaitForHorBasespeed();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(baseSpeed, output->baseSpeed);

  // testing clamping
  baseSpeed = 150;
  velocity.linear.x = baseSpeed;
  PublishTarget(velocity);
  ros::Duration(1).sleep();
  output = WaitForHorBasespeed();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(100, output->baseSpeed);
}

TEST_F(PidAggregatorTest, LinearStrSpeed) {
  double baseSpeed = -69;
  geometry_msgs::Twist velocity;
  velocity.linear.y = baseSpeed;
  PublishTarget(velocity);
  ros::Duration(1).sleep();
  auto output = WaitForStrBasespeed();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(baseSpeed, output->baseSpeed);
}

TEST_F(PidAggregatorTest, LinearVerSpeed) {
  double baseSpeed = -69;
  geometry_msgs::Twist velocity;
  velocity.linear.z = baseSpeed;
  PublishTarget(velocity);
  ros::Duration(1).sleep();
  auto output = WaitForVerBasespeed();
  ASSERT_TRUE(output ==
              NULL);  // no base speed published since depth controller running
}

TEST_F(PidAggregatorTest, ErrorFeedback) {
  // setup state
  au_core::DynamicsState state;
  state.pose.position.x = state.pose.position.y = 0;
  state.pose.position.z = 2.3;
  Eigen::Quaterniond q1 = au_core::rpyToQuat(Eigen::Vector3d(0.3, -0.5, 1.2));
  tf::quaternionEigenToMsg(q1, state.pose.orientation);

  // setup target
  au_core::DynamicsState target;
  target.pose.position.x = target.pose.position.y = 1;  // should have no affect
  target.pose.position.z = 2.0;
  Eigen::Quaterniond q2 = au_core::rpyToQuat(Eigen::Vector3d(0.0, 0.0, M_PI_2));
  tf::quaternionEigenToMsg(q2, target.pose.orientation);

  PublishTarget(target.pose.position, target.pose.orientation, target.velocity);
  auto temp = WaitForDepthTarget();  // make sure the target was sent
  PublishState(state.pose.position, state.pose.orientation);

  // calculate error
  auto output = WaitForError();
  ASSERT_TRUE(output != NULL);
  EXPECT_DOUBLE_EQ(output->pose.position.x, 0);
  EXPECT_DOUBLE_EQ(output->pose.position.y, 0);
  EXPECT_NEAR(output->pose.position.z, -0.3, 0.01);
  Eigen::Quaterniond errorQ;
  tf::quaternionMsgToEigen(output->pose.orientation, errorQ);
  EXPECT_QUATERNION_EQ(errorQ, Eigen::Quaterniond(0.948, -0.097, 0.267, 0.140));
  EXPECT_DOUBLE_EQ(output->velocity.linear.x, 0);
  EXPECT_DOUBLE_EQ(output->velocity.linear.y, 0);
  EXPECT_DOUBLE_EQ(output->velocity.linear.z, 0);
  EXPECT_DOUBLE_EQ(output->velocity.angular.x, 0);
  EXPECT_DOUBLE_EQ(output->velocity.angular.y, 0);
  EXPECT_DOUBLE_EQ(output->velocity.angular.z, 0);
}

}  // namespace pid_control

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pid_aggregator_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
