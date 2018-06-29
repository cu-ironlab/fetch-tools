#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <fetch_custom_msgs/CartesianControls.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>

#include <time.h>
#include <stdio.h>

float control_signals [3] = {0.0, 0.0, 0.0};
bool controls_updated = false;
bool trajectory_active = false;

static const std::string PLANNING_GROUP = "arm_with_torso";
moveit::planning_interface::MoveGroup* move_group;
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group;

void addCollisionObjects()
{
	//TODO: load in objects from an XML file

	
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	//front ground
	moveit_msgs::CollisionObject c_o1;
	c_o1.header.frame_id = move_group->getPlanningFrame();
	c_o1.id = "front_ground";
	shape_msgs::SolidPrimitive primitive1;
	primitive1.type = primitive1.BOX;
	primitive1.dimensions.resize(3);
	primitive1.dimensions[0] = 2;
	primitive1.dimensions[1] = 2;
	primitive1.dimensions[2] = 2;
	geometry_msgs::Pose box_pose1;
	box_pose1.orientation.w = 1.0;
	box_pose1.position.x =  1.1;
	box_pose1.position.y = 0.0;
	box_pose1.position.z =  -1.0;
	c_o1.primitives.push_back(primitive1);
	c_o1.primitive_poses.push_back(box_pose1);
	c_o1.operation = c_o1.ADD;

	collision_objects.push_back(c_o1);

	//back ground
	moveit_msgs::CollisionObject c_o2;
	c_o2.header.frame_id = move_group->getPlanningFrame();
	c_o2.id = "back_ground";
	shape_msgs::SolidPrimitive primitive2;
	primitive2.type = primitive2.BOX;
	primitive2.dimensions.resize(3);
	primitive2.dimensions[0] = 2;
	primitive2.dimensions[1] = 2;
	primitive2.dimensions[2] = 2;
	geometry_msgs::Pose box_pose2;
	box_pose2.orientation.w = 1.0;
	box_pose2.position.x =  -1.2;
	box_pose2.position.y = 0.0;
	box_pose2.position.z =  -1.0;
	c_o2.primitives.push_back(primitive2);
	c_o2.primitive_poses.push_back(box_pose2);
	c_o2.operation = c_o2.ADD;

	collision_objects.push_back(c_o2);

	//left ground
	moveit_msgs::CollisionObject c_o3;
	c_o3.header.frame_id = move_group->getPlanningFrame();
	c_o3.id = "left_ground";
	shape_msgs::SolidPrimitive primitive3;
	primitive3.type = primitive3.BOX;
	primitive3.dimensions.resize(3);
	primitive3.dimensions[0] = 2;
	primitive3.dimensions[1] = 2;
	primitive3.dimensions[2] = 2;
	geometry_msgs::Pose box_pose3;
	box_pose3.orientation.w = 1.0;
	box_pose3.position.x =  0.0;
	box_pose3.position.y = 1.2;
	box_pose3.position.z =  -1.0;
	c_o3.primitives.push_back(primitive3);
	c_o3.primitive_poses.push_back(box_pose3);
	c_o3.operation = c_o3.ADD;

	collision_objects.push_back(c_o3);

	//right_ground
	moveit_msgs::CollisionObject c_o4;
	c_o4.header.frame_id = move_group->getPlanningFrame();
	c_o4.id = "front_ground";
	shape_msgs::SolidPrimitive primitive4;
	primitive4.type = primitive4.BOX;
	primitive4.dimensions.resize(3);
	primitive4.dimensions[0] = 2;
	primitive4.dimensions[1] = 2;
	primitive4.dimensions[2] = 2;
	geometry_msgs::Pose box_pose4;
	box_pose4.orientation.w = 1.0;
	box_pose4.position.x =  0.0;
	box_pose4.position.y = -1.2;
	box_pose4.position.z =  -1.0;
	c_o4.primitives.push_back(primitive4);
	c_o4.primitive_poses.push_back(box_pose4);
	c_o4.operation = c_o4.ADD;

	collision_objects.push_back(c_o4);
	
	planning_scene_interface->addCollisionObjects(collision_objects);
}

void updateControlSignal(const fetch_custom_msgs::CartesianControls::ConstPtr& msg)
{
	if(control_signals[0] != msg->x_axis || control_signals[1] != msg->y_axis || control_signals[2] != msg->z_axis)
	{
		control_signals[0] = msg->x_axis;
		control_signals[1] = msg->y_axis;
		control_signals[2] = msg->z_axis;
		controls_updated = true;
	}
}

void updateTrajectoryStatus(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg)
{
	//msg received means trajectory is complete
	//can dig into message to find result if needed later
	trajectory_active = false;
}

void getTarget(geometry_msgs::Pose* current_pose, geometry_msgs::Pose* target_pose)
{
	//copy orientation
	target_pose->orientation.w = current_pose->orientation.w;
	target_pose->orientation.x = current_pose->orientation.x;
	target_pose->orientation.y = current_pose->orientation.y;
	target_pose->orientation.z = current_pose->orientation.z;
	//update goal to distant point along ray of control direction
	target_pose->position.x = current_pose->position.x + control_signals[0]*2.0;
	target_pose->position.y = current_pose->position.y + control_signals[1]*2.0;
	target_pose->position.z = current_pose->position.z + control_signals[2]*2.0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_keyboard_controller");
	ros::NodeHandle nh;
	ros::ServiceClient executeKnownTrajectoryServiceClient = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
	ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("controller_debug", 1000);
	char buffer [60];

	move_group = new moveit::planning_interface::MoveGroup(PLANNING_GROUP);
	planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	robot_trajectory::RobotTrajectory rt_planner(move_group->getRobotModel(), PLANNING_GROUP);
	trajectory_processing::IterativeParabolicTimeParameterization time_planner;

	addCollisionObjects();
	ros::Subscriber control_sub = nh.subscribe("control_signal", 1, updateControlSignal);
	ros::Subscriber status_sub = nh.subscribe("arm_with_torso_controller/follow_joint_trajectory/result", 1, updateTrajectoryStatus);
	ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::Rate r(30); //update check for new controls at 30 hz
	geometry_msgs::Pose t_pose;
	std_msgs::String msg;
	geometry_msgs::PoseStamped c_pose;
	while(ros::ok())
	{
		if(controls_updated)
		{
			move_group->stop();
			if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0))
			{
				//set target based on current pose and control inputs
				c_pose = move_group->getCurrentPose("wrist_roll_link");
				getTarget(&(c_pose.pose), &t_pose);
				
				 // set waypoints for which to compute path
			    std::vector<geometry_msgs::Pose> waypoints;
			    waypoints.push_back(c_pose.pose);
			    waypoints.push_back(t_pose);
			    moveit_msgs::ExecuteKnownTrajectory srv;

			    // compute cartesian path
			    double ret = move_group->computeCartesianPath(waypoints, 0.1, 10000, srv.request.trajectory, true); //the two magic numbers here are allowed distance between points (meters) and configuration space jump distance (units?)

				sprintf(buffer, "Plan found: %f", ret);
		        msg.data = buffer;
		        chatter_pub.publish(msg);
			    if(ret <= 0.0){
			        // no path could be computed or all paths caused collisions
			        ROS_ERROR("Unable to compute Cartesian path!");
			        sprintf(buffer, "%s", "No path found!\n");
			        msg.data = buffer;
			        chatter_pub.publish(msg);
			    }
			    else
			    {
			    	//add time stamps to trajectory to control velocity
			    	rt_planner.setRobotTrajectoryMsg(*(move_group->getCurrentState()), srv.request.trajectory);
				    time_planner.computeTimeStamps(rt_planner, 0.1, 0.5);
				    rt_planner.getRobotTrajectoryMsg(srv.request.trajectory);

				    // send trajectory to arm controller
				    srv.request.wait_for_execution = false;
				    executeKnownTrajectoryServiceClient.call(srv);
				    trajectory_active = true;
			    }
			}
			controls_updated = false;
		}
		else
		{
			if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0))
			{
				//if controls are being held down but no movement is active, try replanning
				if(!trajectory_active)
				{
					//refresh planning
					controls_updated = true;
				}

				//TODO: sometimes errors are caused by the start state of a trajectory shifting slightly. This bug is somewhat rare, but it cannot be detected in the current program state
				//--need to find someway to detect these errors and consequently replan
			}
		}
		r.sleep();
	}
	move_group->stop();
}