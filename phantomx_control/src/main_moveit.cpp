#include <ros/ros.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
  
  // Start a ROS spinning thread (required for processing callbacks while moveit is blocking)
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("arm");  
  
  ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  

  group.setGoalPositionTolerance(0.04);
  group.setGoalOrientationTolerance(0.1);
  group.allowReplanning(true);
  
  // Planning to a joint-space goal
  
  ROS_INFO("Planing to a joint-space goal");
  
//   group.setJointValueTarget({M_PI/2,M_PI/2,0,0,0});
  
  // Get current joint values
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

  // Set a new target
  group_variable_values[0] = -1.0;
  group_variable_values[1] = -M_PI/2;
  group.setJointValueTarget(group_variable_values);
  group.move(); // we do not test before, just move if possible ;-)
                // move is a blocking call

  // now back to default
  
  ROS_INFO("And back to default conf...");
  
  std::fill(group_variable_values.begin(), group_variable_values.end(), 0.0);
  group.setJointValueTarget(group_variable_values);
  group.move(); // we do not test before, just move if possible ;-)
  
  // Planning to a task-space end effector goal
    ROS_INFO("Planing to a task-space goal");  

  geometry_msgs::Pose target_pose;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  target_pose.position.x = 0.05;
  target_pose.position.y = 0.05;
  target_pose.position.z = 0.7;
  group.setPoseTarget(target_pose);
  
  // here we first check if we can find a plan
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
 
  if (success)
  {
      group.move();
  }
  else
  {
      ROS_ERROR("Cannot find a valid plan...");
  }
  
  

  
  
//   moveit::planning_interface::MoveGroup::Plan my_plan;
//   bool success = group.plan(my_plan);
  
  
//   geometry_msgs::Pose target_pose1;
//   target_pose1.orientation.w = 1.0;
//   target_pose1.position.x = 0.2;
//   target_pose1.position.y = 0.2;
//   target_pose1.position.z = 0.2;
//   group.setPoseTarget(target_pose1);
  




  ros::spin();

  return 0;
}