import math
from enum import IntEnum
import rospy
from tf import transformations as tf
from au_core.msg import DynamicsState


class ValueType(IntEnum):
    NAN = 0
    ABSOLUTE = 1
    REL_STATE = 2
    REL_TARGET = 3


class _Settable:
    def __init__(self, is_angle=False):
        self._value = 0
        self._angle = is_angle
        self._valueType = ValueType.NAN

    def set_value(self, value=0, val_type=ValueType.ABSOLUTE):
        self._value = value
        self._valueType = val_type

    def get_value(self, state=float('nan'), target=float('nan')):
        output = float('nan')
        if self._valueType == ValueType.ABSOLUTE:
            output = self._value
        elif self._valueType == ValueType.REL_STATE:
            output = self._value + state
        elif self._valueType == ValueType.REL_TARGET:
            output = self._value + target

        if self._angle and not math.isnan(output):
            return self._normalize_angle(output)
        else:
            return output

    @staticmethod
    def _normalize_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


class RobotStateCommand:
    def __init__(self):
        self._x = _Settable()
        self._y = _Settable()
        self._z = _Settable()
        self._roll = _Settable(is_angle=True)
        self._pitch = _Settable(is_angle=True)
        self._yaw = _Settable(is_angle=True)
        self._lin_vel_x = _Settable()
        self._lin_vel_y = _Settable()
        self._lin_vel_z = _Settable()
        self._ang_vel_x = _Settable()
        self._ang_vel_y = _Settable()
        self._ang_vel_z = _Settable()

    def set_x(self, val=0, val_type=ValueType.ABSOLUTE):
        self._x.set_value(value=val, val_type=val_type)
        return self

    def set_y(self, val=0, val_type=ValueType.ABSOLUTE):
        self._y.set_value(value=val, val_type=val_type)
        return self

    def set_z(self, val=0, val_type=ValueType.ABSOLUTE):
        self._z.set_value(value=val, val_type=val_type)
        return self

    def set_roll(self, val=0, val_type=ValueType.ABSOLUTE):
        self._roll.set_value(value=val, val_type=val_type)
        return self

    def set_pitch(self, val=0, val_type=ValueType.ABSOLUTE):
        self._pitch.set_value(value=val, val_type=val_type)
        return self

    def set_yaw(self, val=0, val_type=ValueType.ABSOLUTE):
        self._yaw.set_value(value=val, val_type=val_type)
        return self

    def set_vel_lin_x(self, val=0, val_type=ValueType.ABSOLUTE):
        self._lin_vel_x.set_value(value=val, val_type=val_type)
        return self

    def set_vel_lin_y(self, val=0, val_type=ValueType.ABSOLUTE):
        self._lin_vel_y.set_value(value=val, val_type=val_type)
        return self

    def set_vel_lin_z(self, val=0, val_type=ValueType.ABSOLUTE):
        self._lin_vel_z.set_value(value=val, val_type=val_type)
        return self

    def set_vel_ang_x(self, val=0, val_type=ValueType.ABSOLUTE):
        self._ang_vel_x.set_value(value=val, val_type=val_type)
        return self

    def set_vel_ang_y(self, val=0, val_type=ValueType.ABSOLUTE):
        self._ang_vel_y.set_value(value=val, val_type=val_type)
        return self

    def set_vel_ang_z(self, val=0, val_type=ValueType.ABSOLUTE):
        self._ang_vel_z.set_value(value=val, val_type=val_type)
        return self

    def set_pose_nans(self):
        return self.set_x(val_type=ValueType.NAN) \
            .set_y(val_type=ValueType.NAN) \
            .set_z(val_type=ValueType.NAN) \
            .set_roll(val_type=ValueType.NAN) \
            .set_pitch(val_type=ValueType.NAN) \
            .set_yaw(val_type=ValueType.NAN)

    def set_vel_nans(self):
        return self.set_vel_lin_x(val_type=ValueType.NAN) \
            .set_vel_lin_y(val_type=ValueType.NAN) \
            .set_vel_lin_z(val_type=ValueType.NAN) \
            .set_vel_ang_x(val_type=ValueType.NAN) \
            .set_vel_ang_y(val_type=ValueType.NAN) \
            .set_vel_ang_z(val_type=ValueType.NAN)

    def set_all_nans(self):
        return self.set_pose_nans().set_vel_nans()

    def set_orientation_zero_abs(self):
        return self.set_roll(val_type=ValueType.ABSOLUTE) \
            .set_pitch(val_type=ValueType.ABSOLUTE) \
            .set_yaw(val_type=ValueType.ABSOLUTE)

    def set_pose_zero_abs(self):
        return self.set_x(val_type=ValueType.ABSOLUTE) \
            .set_y(val_type=ValueType.ABSOLUTE) \
            .set_z(val_type=ValueType.ABSOLUTE) \
            .set_orientation_zero_abs()

    def set_vel_zero_abs(self):
        return self.set_vel_lin_x(val_type=ValueType.ABSOLUTE) \
            .set_vel_lin_y(val_type=ValueType.ABSOLUTE) \
            .set_vel_lin_z(val_type=ValueType.ABSOLUTE) \
            .set_vel_ang_x(val_type=ValueType.ABSOLUTE) \
            .set_vel_ang_y(val_type=ValueType.ABSOLUTE) \
            .set_vel_ang_z(val_type=ValueType.ABSOLUTE)

    def set_all_zero_abs(self):
        return self.set_pose_zero_abs().set_vel_zero_abs()

    def set_pose_zero_state(self):
        return self.set_x(val_type=ValueType.REL_STATE) \
            .set_y(val_type=ValueType.REL_STATE) \
            .set_z(val_type=ValueType.REL_STATE) \
            .set_roll(val_type=ValueType.REL_STATE) \
            .set_pitch(val_type=ValueType.REL_STATE) \
            .set_yaw(val_type=ValueType.REL_STATE)

    def set_vel_zero_state(self):
        return self.set_vel_lin_x(val_type=ValueType.REL_STATE) \
            .set_vel_lin_y(val_type=ValueType.REL_STATE) \
            .set_vel_lin_z(val_type=ValueType.REL_STATE) \
            .set_vel_ang_x(val_type=ValueType.REL_STATE) \
            .set_vel_ang_y(val_type=ValueType.REL_STATE) \
            .set_vel_ang_z(val_type=ValueType.REL_STATE)

    def set_all_zero_state(self):
        return self.set_pose_zero_state().set_vel_zero_state()

    def set_pose_zero_target(self):
        return self.set_x(val_type=ValueType.REL_TARGET) \
            .set_y(val_type=ValueType.REL_TARGET) \
            .set_z(val_type=ValueType.REL_TARGET) \
            .set_roll(val_type=ValueType.REL_TARGET) \
            .set_pitch(val_type=ValueType.REL_TARGET) \
            .set_yaw(val_type=ValueType.REL_TARGET)

    def set_vel_zero_target(self):
        return self.set_vel_lin_x(val_type=ValueType.REL_TARGET) \
            .set_vel_lin_y(val_type=ValueType.REL_TARGET) \
            .set_vel_lin_z(val_type=ValueType.REL_TARGET) \
            .set_vel_ang_x(val_type=ValueType.REL_TARGET) \
            .set_vel_ang_y(val_type=ValueType.REL_TARGET) \
            .set_vel_ang_z(val_type=ValueType.REL_TARGET)

    def set_all_zero_target(self):
        return self.set_pose_zero_target().set_vel_zero_target()

    def build(self, state=None, target=None):
        new_target = DynamicsState()
        new_target.pose.position.x = self._x.get_value(
            state=state.pose.position.x, target=target.pose.position.x)
        new_target.pose.position.y = self._y.get_value(
            state=state.pose.position.y, target=target.pose.position.y)
        new_target.pose.position.z = self._z.get_value(
            state=state.pose.position.z, target=target.pose.position.z)

        state_rpy = tf.euler_from_quaternion(
            (state.pose.orientation.x, state.pose.orientation.y,
             state.pose.orientation.z, state.pose.orientation.w))
        target_rpy = tf.euler_from_quaternion(
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w))

        new_target_rpy = [0, 0, 0]
        new_target_rpy[0] = self._roll.get_value(
            state=state_rpy[0], target=target_rpy[0])
        new_target_rpy[1] = self._pitch.get_value(
            state=state_rpy[1], target=target_rpy[1])
        new_target_rpy[2] = self._yaw.get_value(
            state=state_rpy[2], target=target_rpy[2])
        new_target_orientation = tf.quaternion_from_euler(
            new_target_rpy[0], new_target_rpy[1], new_target_rpy[2])

        new_target.pose.orientation.x = new_target_orientation[0]
        new_target.pose.orientation.y = new_target_orientation[1]
        new_target.pose.orientation.z = new_target_orientation[2]
        new_target.pose.orientation.w = new_target_orientation[3]

        new_target.velocity.linear.x = self._lin_vel_x.get_value(
            state=state.velocity.linear.x, target=target.velocity.linear.x)
        new_target.velocity.linear.y = self._lin_vel_y.get_value(
            state=state.velocity.linear.y, target=target.velocity.linear.y)
        new_target.velocity.linear.z = self._lin_vel_z.get_value(
            state=state.velocity.linear.z, target=target.velocity.linear.z)

        new_target.velocity.angular.x = self._ang_vel_x.get_value(
            state=state.velocity.angular.x, target=target.velocity.angular.x)
        new_target.velocity.angular.y = self._ang_vel_y.get_value(
            state=state.velocity.angular.y, target=target.velocity.angular.y)
        new_target.velocity.angular.z = self._ang_vel_z.get_value(
            state=state.velocity.angular.z, target=target.velocity.angular.z)

        new_target.header.stamp = rospy.Time.now()

        return new_target
