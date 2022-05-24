#!/usr/bin/env python2
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from ifo_gazebo.srv import (
    TestMocapGap,
    TestMocapGapResponse,
    TestMocapSuddenChange,
    TestMocapSuddenChangeResponse,
    TestMocapZeroReading,
    TestMocapZeroReadingResponse,
)

"""
This node "emulates" a motion capture VRPN server by creating topics with 
identical messages to those seen when using the `vrpn_client_ros` package.
See http://wiki.ros.org/vrpn_client_ros. 
"""


class MocapSimulatorNode(object):
    def __init__(self):
        super(MocapSimulatorNode, self).__init__()
        rospy.init_node("vrpn_client_ros")
        self.model_states = None
        self.state_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, callback=self.cb_model_states
        )
        self.wait_for_first_message()
        self.pubs = {}
        self.model_names = self.model_states.name
        self.create_publishers(self.model_names)
        self.test_gap_srv = rospy.Service("vrpn_client_ros/test_gap", TestMocapGap, self.test_mocap_gap)
        self.test_change_srv = rospy.Service(
            "vrpn_client_ros/test_sudden_change", TestMocapSuddenChange, self.test_mocap_sudden_change
        )
        self.test_zero_srv = rospy.Service(
            "vrpn_client_ros/test_zero_reading", TestMocapZeroReading, self.test_mocap_zero_reading
        )

        self.test_gap_flags = {name: False for name in self.model_names}
        self.test_change_flags = {name: False for name in self.model_names}
        self.test_zero_flags = {name: False for name in self.model_names}

    def create_publishers(self, model_names):
        for model in model_names:
            model.replace(" ", "_")
            if model != "ground_plane":
                self.pubs[model] = rospy.Publisher(
                    "vrpn_client_node/" + model + "/pose", PoseStamped, queue_size=1
                )

    def cb_model_states(self, model_states_msg):
        self.timestamp = rospy.Time.now()
        self.model_states = model_states_msg

    def wait_for_first_message(self):
        rate = rospy.Rate(10)
        while self.model_states is None and not rospy.is_shutdown():
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def publish_poses(self):
        states = self.model_states
        timestamp = self.timestamp
        poses = dict(zip(states.name, states.pose))

        for model_name in poses:

            do_gap_test = False
            do_zero_test = False
            if model_name != "ground_plane":

                if model_name in self.test_gap_flags.keys():
                    do_gap_test = self.test_gap_flags[model_name]

                if model_name in self.test_zero_flags.keys():
                    do_zero_test = self.test_zero_flags[model_name]                   

                if not do_gap_test:
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = timestamp

                    if not do_zero_test:
                        # pose message gets initialized to zero by default
                        pose = poses[model_name]
                        pose_msg.pose.position = pose.position
                        pose_msg.pose.orientation = pose.orientation

                    if not model_name in self.pubs:
                        self.create_publishers([model_name])

                    pub = self.pubs[model_name]
                    pub.publish(pose_msg)

    def test_mocap_gap(self, *args):
        """
        Simulates a mocap blackout (gap in data) for a short duration.
        """
        # TODO: could potentially accept a duration argument, as well as body name

        duration = 2  # seconds
        self.test_gap_flags = {name: False for name in self.model_states.name}
        rospy.logwarn("Performing mocap gap test!")

        for key in self.test_gap_flags.keys():
            self.test_gap_flags[key] = True

        rospy.sleep(duration)

        for key in self.test_gap_flags.keys():
            self.test_gap_flags[key] = False

        return TestMocapGapResponse(True)

    def test_mocap_sudden_change(self, *args):
        # TODO
        pass

    def test_mocap_zero_reading(self, *args):
        """
        Simulates a "zero reading" from the mocap, which is a behaviour we
        sometimes see during a data gap.
        """
        duration = 2  # seconds
        self.test_zero_flags = {name: False for name in self.model_states.name}
        rospy.logwarn("Performing zero reading test!")

        for key in self.test_zero_flags.keys():
            self.test_zero_flags[key] = True

        rospy.sleep(duration)

        for key in self.test_zero_flags.keys():
            self.test_zero_flags[key] = False

        return TestMocapZeroReadingResponse(True)

    def start(self):
        rate = rospy.Rate(100)  # Hz TODO. Make a parameter.
        while not rospy.is_shutdown():
            self.publish_poses()
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == "__main__":
    node = MocapSimulatorNode()
    node.start()
    rospy.spin()
