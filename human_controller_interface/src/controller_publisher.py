#!/usr/bin/env python

import rospy
import xml
import sys
import KeyboardController
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import CartesianControls

def start_node():
	myargv = rospy.myargv(argv=sys.argv)
	if(len(myargv) < 4):
		print("ERROR: XML files for controller spec not given")
		sys.exit(-1)
	controller_spec = xml.etree.ElementTree.parse(myargv[1])
	keyboard_spec = xml.etree.ElementTree.parse(myargv[2])
	kc = KeyboardController.DiscreteKeyboardController(control_spec, keyboard_spec)
	kc.start_listener()
	return kc, int(myargv[3])


if __name__ == '__main__':
    rospy.init_node("Robot_Controller")
    pub = rospy.Publisher('control_signal',	CartesianControls, queue_size=1)
    kc, hz = start_node()
    rate = rospy.Rate(hz)
    while (not rospy.is_shutdown()):
    	controls = kc.get_all_controls()
    	control_msg = CartesianControls()
    	control_msg.x_axis = controls["x_axis"]["input"]
    	control_msg.y_axis = controls["y_axis"]["input"]
    	control_msg.z_axis = controls["z_axis"]["input"]
    	pub.publish(control_msg)
    	rate.sleep()