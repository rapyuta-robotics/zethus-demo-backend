// C++
#include <string>
#include <random>
#include <iostream>
#include <fstream>
#include <time.h>

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

// Visual tools
#include <moveit_visual_tools/moveit_visual_tools.h>

//  Eigen
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rvt = rviz_visual_tools;
namespace mvt = moveit_visual_tools;

int main(int argc, char** argv)
{
	ROS_INFO_STREAM("Start");
	std::string name_ = "moveit_msgs_publisher";
	ros::init(argc, argv, name_);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh_("~/moveit_msgs_publisher");

	ROS_INFO_STREAM("Start)");

	std::string robot_description = "robot_description";
	std::string planning_scene_topic = "planning_scene";
	std::string moveit_visual_marker_topic = "moveit_visual_marker";
	std::string world_frame_ = "world";

	robot_model::RobotModelPtr robot_model_;
	robot_model_loader::RobotModelLoaderPtr robot_model_loader;
	robot_model_loader.reset(new robot_model_loader::RobotModelLoader(robot_description));
	// Load the robot model
	robot_model_ = robot_model_loader->getModel();  // Get a shared pointer to the robot

	// Create goal robot state
	moveit::core::RobotStatePtr robot_state_;
	ROS_INFO_STREAM_NAMED(name_, "Loading robot start and goal state");
	robot_state_.reset(new moveit::core::RobotState(robot_model_));
	robot_state_->setToDefaultValues();

	// Transforms
  	boost::shared_ptr<tf::TransformListener> tf_;
	// TransformListener
  	tf_.reset(new tf::TransformListener(nh_));

	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

	// Create the planning scene
	planning_scene::PlanningScenePtr planning_scene;
	planning_scene.reset(new planning_scene::PlanningScene(robot_model_));

	// Allows us to syncronize to Rviz and also publish collision objects to ourselves
	ROS_DEBUG_STREAM_NAMED(name_, "Loading Planning Scene Monitor");
	planning_scene_monitor_.reset(
	  new planning_scene_monitor::PlanningSceneMonitor(planning_scene, robot_model_loader, tf_));
	ROS_ASSERT_MSG(planning_scene_monitor_->getPlanningScene(), "failed to get planning scene");

	planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
	planning_scene_monitor_->setStateUpdateFrequency(100);
	planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
	planning_scene_monitor_->startStateMonitor();
	planning_scene_monitor_->startSceneMonitor(planning_scene_topic);

	// Spin while we wait for the full robot state to become available
	// while (!planning_scene_monitor_->getStateMonitor()->haveCompleteState() && ros::ok())
	// {
	// 	ROS_INFO_STREAM_THROTTLE_NAMED(1, name_, "Waiting for complete state from topic ");
	// }

	// For visualizing things in rviz
	moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

	// Load the Robot Viz Tools for publishing to Rviz
	visual_tools_.reset(new mvt::MoveItVisualTools(world_frame_, moveit_visual_marker_topic, planning_scene_monitor_));
	visual_tools_->setPlanningSceneTopic(planning_scene_topic);
	visual_tools_->loadMarkerPub();
	visual_tools_->deleteAllMarkers();  // clear all old markers
	// visual_tools_->hideRobot();         // show that things have been reset
	visual_tools_->enableBatchPublishing();

	visual_tools_->publishRobotState(robot_state_, rvt::GREEN);
    visual_tools_->trigger();

	// Shutdown
  	ros::waitForShutdown();
	return 0;
}