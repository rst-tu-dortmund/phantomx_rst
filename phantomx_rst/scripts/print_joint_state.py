#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy
from sensor_msgs.msg import JointState



def callback(data):
  try:
    # Test if we have the message containing all other joint values
    # I just hardcode the names, since we do not have the right ordering in the msg ;-)
    shoulder_pan_idx = data.name.index("arm_shoulder_pan_joint") 
    shoulder_lift_idx = data.name.index("arm_shoulder_lift_joint") 
    elbow_flex_idx = data.name.index("arm_elbow_flex_joint") 
    wrist_flex_idx = data.name.index("arm_wrist_flex_joint") 
    gripper_idx = data.name.index("gripper_joint") 

    rospy.loginfo("q: [%f, %f, %f, %f], gripper: %f", data.position[shoulder_pan_idx], data.position[shoulder_lift_idx], data.position[elbow_flex_idx], data.position[wrist_flex_idx], data.position[gripper_idx])

  except ValueError:
    pass





def print_joints():
  rospy.init_node("print_joint_state")
  rospy.Subscriber("joint_states", JointState, callback)
  rospy.spin()




if __name__ == '__main__':
    print_joints()

