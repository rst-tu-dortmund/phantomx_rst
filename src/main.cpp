/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/


#include <phantomx_rst/phantomx_interface.h>




// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
  
  phantomx::PhantomXControl robot;
  robot.initialize();

  robot.setJointsDefault();
  
  phantomx::JointVector joints;
  joints.setZero();
  joints[0] = M_PI/2;
  joints[1] = M_PI/2;
//   robot.setJoints(joints, 1.5);
    
  
  
  trajectory_msgs::JointTrajectoryPoint pt;
  
  // create trajectory to the single goal
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = robot.getJointNames();
  goal.trajectory.header.stamp = ros::Time::now();
  
  goal.trajectory.points.resize(1);
  goal.trajectory.points.front().time_from_start = ros::Duration(0);
  goal.trajectory.points.front().positions.assign(joints.data(), joints.data()+joints.rows());
  
  phantomx::JointVector vels;
  vels.setZero();
  vels[0] = 0.1;
  vels[1] = 0.2;
  
  goal.trajectory.points.front().velocities.assign(vels.data(), vels.data()+vels.rows());

  robot.setJointTrajectory(goal);
//   robot.setJointsDefault();
  
// trajectory_msgs::JointTrajectoryPoint pt;
// pt.positions;
  
  /*
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_ac("/gripper_controller/gripper_action", true);
  
  ROS_INFO("Waiting for gripper action server to start.");
  gripper_ac.waitForServer();
  
  ROS_INFO("Gripper action server started, sending goal.");
  
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.01;
  
  gripper_ac.sendGoal(gripper_goal);
 
  
  //wait for the action to return
  bool finished_before_timeout = gripper_ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = gripper_ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  
  */
  
  return 0;
}

