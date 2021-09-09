#!/usr/bin/env python2
import rospy 
from gazebo_msgs.msg import ModelStates 
from geometry_msgs.msg import PoseStamped
"""
This node "emulates" a motion capture VRPN server by creating topics with 
identical messages to those seen when using the `vrpn_client_ros` package.
See http://wiki.ros.org/vrpn_client_ros. 
"""

class MocapSimulatorNode(object):
    def __init__(self):
        super(MocapSimulatorNode, self).__init__()
        rospy.init_node('vrpn_client_ros')
        self.model_states = None
        self.state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                          callback=self.cb_model_states)
        self.wait_for_first_message()
        self.pubs = {}
        model_names = self.model_states.name
        self.create_publishers(model_names)
        
                                                   
    def create_publishers(self, model_names):
        for model in model_names:
            if model != "ground_plane":
                self.pubs[model] = rospy.Publisher(
                                    'vrpn_client_node/' + model + '/pose',
                                    PoseStamped, queue_size=1
                                    )

    def wait_for_first_message(self):
        rate = rospy.Rate(10)
        while self.model_states is None and not rospy.is_shutdown():
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

            
    def cb_model_states(self, model_states_msg):
        self.timestamp = rospy.Time.now()
        self.model_states = model_states_msg

    def publish_poses(self):
        states = self.model_states
        timestamp = self.timestamp
        poses = dict(zip(states.name, states.pose))

        for model_name in poses:
            if model_name != 'ground_plane':
                pose_msg = PoseStamped()
                pose_msg.header.stamp = timestamp
                pose = poses[model_name]
                pose_msg.pose.position = pose.position
                pose_msg.pose.orientation = pose.orientation

                if not model_name in self.pubs:
                    self.create_publishers([model_name])

                pub = self.pubs[model_name]
                pub.publish(pose_msg)

    def start(self):
        rate = rospy.Rate(100) #Hz TODO. Make a parameter.
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