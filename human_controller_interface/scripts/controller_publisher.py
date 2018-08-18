#!/usr/bin/env python
import rospy
import xml
import sys
import src.InputController as InputController
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from fetch_custom_msgs.msg import CartesianControlsWithGripper

def start_node():
	myargv = rospy.myargv(argv=sys.argv)
	if(len(myargv) < 5):
		print("ERROR: XML files for controller spec and/or control type not given")
		sys.exit(-1)
	controller_spec = xml.etree.ElementTree.parse(myargv[1])
	button_spec = xml.etree.ElementTree.parse(myargv[2])
	controller_type = str(myargv[3])
	if(controller_type == "keyboard"):
		#TODO: update this to include with gripper
		kc = InputController.DiscreteKeyboardController(controller_spec, button_spec)
	elif(controller_type == "xbox"):
		kc = InputController.CartesianWGXboxController(controller_spec, button_spec)
	else:
		print("ERROR: unsupported control type")
		sys.exit(-1)
	kc.start_listener()
	return kc, int(myargv[4])

if __name__ == '__main__':
	rospy.init_node("Robot_Controller")
	pub = rospy.Publisher('/sa_experiment/control_signal', CartesianControlsWithGripper, queue_size=1)
	kc, hz = start_node()
	rate = rospy.Rate(hz)
	while (not rospy.is_shutdown()):
		controls = kc.get_all_controls()
		control_msg = CartesianControlsWithGripper()
		control_msg.x_axis = controls["x_axis"]["input"]
		control_msg.y_axis = controls["y_axis"]["input"]
		control_msg.z_axis = controls["z_axis"]["input"]
		control_msg.gripper = controls["gripper"]["input"]
		pub.publish(control_msg)
		rate.sleep()
	kc.stop_listener()