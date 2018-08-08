#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <fetch_custom_msgs/CartesianControls.h>
#include <fetch_custom_msgs/ObstacleList.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/String.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>

#include <time.h>
#include <stdio.h>
#include <cmath>

const int TRAJECTORY_INACTIVE = 0;
const int TRAJECTORY_PENDING = 1;
const int TRAJECTORY_ACTIVE = 2;

float control_signals [3] = {0.0, 0.0, 0.0};
bool controls_updated = false;
int trajectory_status = 0;
int last_movement_axis = -1;

static const std::string PLANNING_GROUP = "arm_with_torso";
moveit::planning_interface::MoveGroup* move_group;
moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group;
geometry_msgs::Pose starting_pose;

void addCollisionObjects(ros::ServiceClient sc)
{
	fetch_custom_msgs::ObstacleList srv;
	ros::service::waitForService("/get_obstacle_objects", -1);
	srv.request.request_string = "ObstacleList";
	if(!sc.call(srv))
	{
		//Error in service call
		return;
	}

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	int num_obstacles = srv.response.num_obstacles;
	int i = 0;
	for(i = 0; i < num_obstacles; ++i)
	{
		moveit_msgs::CollisionObject c;
		c.header.frame_id = move_group->getPlanningFrame();
		c.id = srv.response.obstacle_names[i];
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[0] = srv.response.obstacle_coordinates[i*7 + 0];
		primitive.dimensions[1] = srv.response.obstacle_coordinates[i*7 + 1];
		primitive.dimensions[2] = srv.response.obstacle_coordinates[i*7 + 2];
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = srv.response.obstacle_coordinates[i*7 + 3];
		box_pose.position.x =  srv.response.obstacle_coordinates[i*7 + 4];
		box_pose.position.y = srv.response.obstacle_coordinates[i*7 + 5];
		box_pose.position.z =  srv.response.obstacle_coordinates[i*7 + 6];
		c.primitives.push_back(primitive);
		c.primitive_poses.push_back(box_pose);
		c.operation = c.ADD;

		collision_objects.push_back(c);
	}
	
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

void updateTrajectoryGoal(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
	trajectory_status = TRAJECTORY_ACTIVE;
}

void updateTrajectoryStatus(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& msg)
{
	//msg received means trajectory is complete
	//can dig into message to find result if needed later
	trajectory_status = TRAJECTORY_INACTIVE;
}

void getTarget(geometry_msgs::Pose* target_pose, geometry_msgs::Pose ideal_pose, geometry_msgs::Pose current_pose)
{
	target_pose->position.x = ideal_pose.position.x;
	target_pose->position.y = ideal_pose.position.y;
	target_pose->position.z = ideal_pose.position.z;
	if(control_signals[0] != 0.0)
	{
		target_pose->position.x += control_signals[0]*1.0;
		last_movement_axis = 0;
	}
	if(control_signals[1] != 0.0)
	{
		target_pose->position.y += control_signals[1]*1.0;
		last_movement_axis = 1;
	}
	if(control_signals[2] != 0.0)
	{
		target_pose->position.z += control_signals[2]*1.0;
		last_movement_axis = 2;
	}
}

void updateIdealPose(geometry_msgs::Pose* ideal_pose, geometry_msgs::Pose current_pose)
{
	if(last_movement_axis == 0)
	{
		ideal_pose->position.x = current_pose.position.x;
	} else if (last_movement_axis == 1)
	{
		ideal_pose->position.y = current_pose.position.y;
	} else if (last_movement_axis == 2)
	{
		ideal_pose->position.z = current_pose.position.z;
	}
	last_movement_axis = -1;
}

void screenTrajectory(moveit_msgs::RobotTrajectory* orig_traj)
{
	//tune this to prevent major configuration space changes (which sometimes don't properly screen collisions)
	float distance_cutoff = 3.0;
	std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator first = orig_traj->joint_trajectory.points.begin() + 0;
	std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator last = orig_traj->joint_trajectory.points.begin() + orig_traj->joint_trajectory.points.size();
	int i;
	for(i = 0; i < orig_traj->joint_trajectory.points.size() - 1; ++i)
	{
		float dist_sq = 0.0;
		int j;
		for(j = 0; j < orig_traj->joint_trajectory.points[i].positions.size(); ++j)
		{
			float dist_ = orig_traj->joint_trajectory.points[i+1].positions[j] - orig_traj->joint_trajectory.points[i].positions[j];
			dist_sq += dist_*dist_;
		}
		if(dist_sq > distance_cutoff)
		{
			int size_diff = orig_traj->joint_trajectory.points.size() - (i+1);
			ROS_ERROR("Clipping [%d] points out of trajectory", size_diff);
			last = orig_traj->joint_trajectory.points.begin() + (i+1);
			break;
		}
	}
	std::vector<trajectory_msgs::JointTrajectoryPoint> new_points(first, last);
	orig_traj->joint_trajectory.points = new_points;
}

void waitForMotion()
{
	while(trajectory_status > 0)
	{
		sleep(0.5);
	}
	ROS_ERROR("Trajectory Complete");
}

void moveToStartingPose()
{
	starting_pose.orientation.w = 1.0;
	starting_pose.orientation.x = 0.0;
	starting_pose.orientation.y = 0.0;
	starting_pose.orientation.z = 0.0;

	starting_pose.position.x = 0.4;
	starting_pose.position.y = -0.1;
	starting_pose.position.z = 1.1;

	moveit::planning_interface::MoveGroup::Plan my_plan;
	move_group->setPoseTarget(starting_pose);
	move_group->setPlanningTime(5.0);
	move_group->setMaxAccelerationScalingFactor(0.5);
	move_group->setMaxVelocityScalingFactor(0.2);
	move_group->asyncMove();
	trajectory_status = TRAJECTORY_ACTIVE;
	ROS_ERROR("Waiting on starting position...");

	sleep(5.0);
	waitForMotion();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cartesian_controller");
	ros::NodeHandle nh;
	ros::ServiceClient executeKnownTrajectoryServiceClient = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/execute_kinematic_path");
	ros::Publisher chatter_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_state", 1000);

	move_group = new moveit::planning_interface::MoveGroup(PLANNING_GROUP);
	planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
	joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	robot_trajectory::RobotTrajectory rt_planner(move_group->getRobotModel(), PLANNING_GROUP);
	trajectory_processing::IterativeParabolicTimeParameterization time_planner;

	ros::ServiceClient getObstacleObjectsServiceClient = nh.serviceClient<fetch_custom_msgs::ObstacleList>("/get_obstacle_objects");
	addCollisionObjects(getObstacleObjectsServiceClient);
	ros::Subscriber control_sub = nh.subscribe("automated_control_signal", 1, updateControlSignal);
	ros::Subscriber traj_received = nh.subscribe("arm_with_torso_controller/follow_joint_trajectory/goal", 1, updateTrajectoryGoal);
	ros::Subscriber status_sub = nh.subscribe("arm_with_torso_controller/follow_joint_trajectory/result", 1, updateTrajectoryStatus);
	ros::AsyncSpinner spinner(3);
	spinner.start();
	moveToStartingPose();

	ros::Rate r(10); //update check for new controls at 10 hz
	geometry_msgs::PoseStamped t_pose;
	geometry_msgs::PoseStamped ideal_pose;
	std_msgs::String msg;
	geometry_msgs::PoseStamped c_pose;

	//Save initial behavior
	t_pose = move_group->getCurrentPose("wrist_roll_link");
	ideal_pose = move_group->getCurrentPose("wrist_roll_link");
	while(ros::ok())
	{
		c_pose = move_group->getCurrentPose("wrist_roll_link");
		chatter_pub.publish(c_pose);

		if(controls_updated)
		{
			move_group->stop();
			updateIdealPose(&(ideal_pose.pose), c_pose.pose);
			if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0))
			{
				//set target based on current pose and control inputs
				getTarget(&(t_pose.pose), ideal_pose.pose, c_pose.pose);
				
				 // set waypoints for which to compute path
			    std::vector<geometry_msgs::Pose> waypoints;
			    waypoints.push_back(t_pose.pose);
			    moveit_msgs::ExecuteKnownTrajectory srv;

			    // compute cartesian path
			    double ret = move_group->computeCartesianPath(waypoints, 0.1, 10000, srv.request.trajectory, true); //the two magic numbers here are allowed distance between points (meters) and configuration space jump distance (units?)

			    if(ret <= 0.0){
			        // no path could be computed or all paths caused collisions
			        //ROS_ERROR("Unable to compute Cartesian path!");
			    	;
			    }
			    else
			    {
			    	//add time stamps to trajectory to control velocity
			    	rt_planner.setRobotTrajectoryMsg(*(move_group->getCurrentState()), srv.request.trajectory);
				    time_planner.computeTimeStamps(rt_planner, 0.1, 0.5);
				    rt_planner.getRobotTrajectoryMsg(srv.request.trajectory);

				    screenTrajectory(&srv.request.trajectory);
				    // send trajectory to arm controller
				    srv.request.wait_for_execution = false;
				    executeKnownTrajectoryServiceClient.call(srv);
				    trajectory_status = TRAJECTORY_PENDING;
			    }
			}
			else
			{
				// set waypoints for which to compute path
			    std::vector<geometry_msgs::Pose> waypoints;
			    waypoints.push_back(ideal_pose.pose);
			    moveit_msgs::ExecuteKnownTrajectory srv;

			    // compute cartesian path
			    double ret = move_group->computeCartesianPath(waypoints, 0.1, 10000, srv.request.trajectory, true); //the two magic numbers here are allowed distance between points (meters) and configuration space jump distance (units?)

			    screenTrajectory(&srv.request.trajectory);
			    // send trajectory to arm controller
			    srv.request.wait_for_execution = false;
			    executeKnownTrajectoryServiceClient.call(srv);
			}
			controls_updated = false;
		}
		else
		{
			if(!(control_signals[0] == 0.0 && control_signals[1] == 0.0 && control_signals[2] == 0.0))
			{
				//if controls are being held down but no movement is active, try replanning
				if(trajectory_status == TRAJECTORY_INACTIVE)
				{
					//TODO: maybe don't do this at quite as fast of a rate
					//refresh planning
					controls_updated = true;
				}
				else
				{
					if(trajectory_status == TRAJECTORY_PENDING)
					{
						//if trajectory was sent but not received, replan
						controls_updated = true;
					}
				}
			}
		}
		r.sleep();
	}
	move_group->stop();
}