#!/usr/bin/env python
"""This module implements a state machine that publishes the target linear and angular velocities to the target_linear_velocity and target_angular_velocity topics.
"""


import rospy
import smach
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist


class DriveState(smach.State):
    """This state publishes the target linear and angular velocities"""

    def __init__(self):
        """Constructor of the DriveState class"""
        smach.State.__init__(self, outcomes=["stop"])

    def execute(self, userdata):
        """This function is called when the state is active. It publishes the target linear and angular velocities.

        Returns:
            retrun the next state to transition to
        """
        rospy.loginfo("Entering Drive state...")
        # Set the publish rate of this state
        rate = rospy.Rate(100)  # 100 Hz
        # publish the target linear and angular velocities obtained from the target_twist topic
        global v, omega

        # Publish the target velocities until the signal becomes False
        while not signal_value and not rospy.is_shutdown():
            target_v_pub.publish(v)
            if v.data < 0:
                omega_zero = Float64()
                omega_zero.data = 0.0
                target_omega_pub.publish(omega_zero)
            else:
                target_omega_pub.publish(omega)
            rate.sleep()

        # Transition to the stop state when the signal becomes False
        return "stop"


class StopState(smach.State):
    """This state publishes zero linear and angular velocities to stop the car"""

    def __init__(self):
        """Constructor of the StopState class"""
        smach.State.__init__(self, outcomes=["drive"])

    def execute(self, userdata):
        """This function is called when the state is active. It publishes zero linear and angular velocities to stop the car.

        Returns:
            return the next state to transition to
        """
        rospy.loginfo("Entering Stop state...")

        # Set the publish rate of this state
        rate = rospy.Rate(100)  # 100 Hz

        # Publish zero linear and angular velocities until the signal becomes True
        while signal_value and not rospy.is_shutdown():
            v = Float64()
            v.data = 0.0  # Some value to publish in the stop state
            target_v_pub.publish(v)
            omega = Float64()
            omega.data = 0.0
            target_omega_pub.publish(omega)
            rate.sleep()

        # Transition to the drive state when the signal becomes True
        return "drive"


def target_twist_callback(target_twist_msg):
    """This function is called when a message is received from the target_twist topic. It stores the target linear and angular velocities in global variables.

    Args:
        target_twist_msg: a message of type Twist
    """
    # Store the target linear and angular velocities in global variables
    v.data = target_twist_msg.linear.x
    omega.data = target_twist_msg.angular.z


class TrafficLight:
    """This class implements a traffic light that is used to stop the car when the signal is red.

    Attributes: 
        signal_sub: a subscriber to the perception/traffic_state topic
        target_v_pub: a publisher to the target topic
        last_signal_value: the last value of the signal
        counter: a counter to count the number of times the signal has the same value
    """
    def __init__(self) -> None:
        """ Constructor of the TrafficLight class """
        # initialize the global variable signal_value
        global signal_value
        signal_value = False

        # Create the subscriber to the perception/traffic_state topic
        self.signal_sub = rospy.Subscriber("perception/traffic_state", Bool, self.signal_callback)

        # Create the publisher to the target topic
        self.target_v_pub = rospy.Publisher("target", Bool, queue_size=10)

        # Initialize the last_signal_value and the counter
        self.last_signal_value = False
        self.counter = 0

    def signal_callback(self, signal_msg):
        """This function is called when a message is received from the perception/traffic_state topic. It publishes the signal value to the target topic.

        Args:
            signal_msg: a message of type Bool
        """
        global signal_value
        sig = Bool()
        sig.data = signal_value

        # Publish the signal value to the target topic
        self.target_v_pub.publish(sig)

        # If the signal has the same value as the last time, increase the counter by 1
        if self.last_signal_value == signal_msg.data:
            self.counter += 1
        # If the signal has a different value than the last time, reset the counter and update the last_signal_value
        else:
            self.counter = 0
            self.last_signal_value = signal_msg.data

        # If the signal has the same value for more than 5 times, update the global variable signal_value
        if self.counter > 5:
            signal_value = signal_msg.data
            self.counter = 0


def main():
    """ This function is the main function of this file. It initializes the node, creates the state machine, and executes it. """
    # Initialize the node
    rospy.init_node("smach")

    # Create an instance of the TrafficLight class
    traffic_light = TrafficLight()

    # Create the subscriber to the target_twist topic
    global v, omega
    v = Float64()
    omega = Float64()
    twist_sub = rospy.Subscriber("target_twist", Twist, target_twist_callback)

    # Create the publishers for the target linear and angular velocities
    global target_v_pub, target_omega_pub
    target_v_pub = rospy.Publisher("target_linear_velocity", Float64, queue_size=10)
    target_omega_pub = rospy.Publisher("target_angular_velocity", Float64, queue_size=10)

    # Create the state machine
    sm = smach.StateMachine(outcomes=[])

    with sm:
        smach.StateMachine.add("DRIVE", DriveState(), transitions={"stop": "STOP"})
        smach.StateMachine.add("STOP", StopState(), transitions={"drive": "DRIVE"})

    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    outcome = sm.execute()
    
    rospy.spin()


if __name__ == "__main__":
    main()
