#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class DriveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["stop"])

    def execute(self, userdata):
        rospy.loginfo("Entering Drive state...")
        rate = rospy.Rate(100)  # 100 Hz
        global v, omega

        while not signal_value and not rospy.is_shutdown():
            target_v_pub.publish(v)
            target_omega_pub.publish(omega)
            rate.sleep()

        return "stop"  # Transition to the stop state when the signal becomes False


class StopState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["drive"])

    def execute(self, userdata):
        rospy.loginfo("Entering Stop state...")
        rate = rospy.Rate(100)  # 100 Hz

        while signal_value and not rospy.is_shutdown():
            v = Float64()
            v.data = 0.0  # Some value to publish in the stop state
            target_v_pub.publish(v)
            omega = Float64()
            omega.data = 0.0
            target_omega_pub.publish(omega)
            rate.sleep()

        return "drive"  # Transition to the drive state when the signal becomes True


def target_twist_callback(target_twist_msg):
    v.data = target_twist_msg.linear.x
    omega.data = target_twist_msg.angular.z


class TrafficLight:
    def __init__(self) -> None:
        global signal_value
        signal_value = False
        self.signal_sub = rospy.Subscriber("perception/traffic_state", Bool, self.signal_callback)
        self.target_v_pub = rospy.Publisher("target", Bool, queue_size=10)
        self.last_signal_value = False
        self.counter = 0

    def signal_callback(self, signal_msg):
        global signal_value
        sig=Bool()
        sig.data=signal_value
        self.target_v_pub.publish(sig)
        if self.last_signal_value == signal_msg.data:
            self.counter += 1
        else:
            self.counter = 0
            self.last_signal_value = signal_msg.data
        if self.counter > 5:
            signal_value = signal_msg.data
            self.counter = 0



def main():
    rospy.init_node("smach")

    traffic_light = TrafficLight()

    global v, omega
    v = Float64()
    omega = Float64()
    twist_sub = rospy.Subscriber("target_twist", Twist, target_twist_callback)

    global target_v_pub, target_omega_pub
    target_v_pub = rospy.Publisher("target_linear_velocity", Float64, queue_size=10)
    target_omega_pub = rospy.Publisher("target_angular_velocity", Float64, queue_size=10)

    sm = smach.StateMachine(outcomes=[])

    with sm:
        smach.StateMachine.add("DRIVE", DriveState(), transitions={"stop": "STOP"})
        smach.StateMachine.add("STOP", StopState(), transitions={"drive": "DRIVE"})

    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    outcome = sm.execute()

if __name__ == "__main__":
    main()
