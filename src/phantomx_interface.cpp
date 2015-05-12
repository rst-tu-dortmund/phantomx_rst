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


namespace phantomx
{
  
PhantomXControl::PhantomXControl()
{
}

PhantomXControl::~PhantomXControl()
{
  if (_joints_sub_spinner)
    _joints_sub_spinner->stop();
}
  
void PhantomXControl::initialize()
{
  // instantiate arm action client
  ROS_INFO("Waiting for arm action server to start.");
  _arm_action = make_unique<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("/arm_controller/follow_joint_trajectory", true);
  _arm_action->waitForServer(); // will wait for infinite time

  
  // instantiate gripper action client
  ROS_INFO("Arm action server found. Waiting for gripper action server to start.");
  _gripper_action = make_unique<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>>("/gripper_controller/gripper_action", true);
  _gripper_action->waitForServer(); // will wait for infinite time
  ROS_INFO("All action servers started.");
  
  // setup subscriber (get joint_states in a separate thread)
  ros::NodeHandle n;
  _joints_sub_queue = new ros::CallbackQueue();
  n.setCallbackQueue(_joints_sub_queue);
  _joints_sub = n.subscribe("/joint_states", 1, &PhantomXControl::jointStateCallback, this);
  _joints_sub_spinner = make_unique<ros::AsyncSpinner>(1, _joints_sub_queue);
  ROS_ERROR_COND(!_joints_sub_spinner->canStart(),"Asynchronous spinner for reveiving new joint state messages cannot be started.");
  _joints_sub_spinner->start();

  // setup joint names
  _joint_names_arm = {"arm_shoulder_pan_joint",
		      "arm_shoulder_lift_joint",
		      "arm_elbow_flex_joint",
		      "arm_wrist_flex_joint"};
  for (int i=0; i< _joint_names_arm.size(); ++i)
  {
    _map_joint_to_index[_joint_names_arm.at(i)] = i;
  }
  
  // get joint information (angle limits and max speed)
  for (int i=0; i<_joint_names_arm.size(); ++i)
  {
	std::string param_prefix = "/arbotix/joints/" + _joint_names_arm[i]; // TODO config param
        std::string min_angle_key = param_prefix + "/min_angle";
	std::string max_angle_key = param_prefix + "/max_angle";
	std::string max_speed_key = param_prefix + "/max_speed";
	
	if (!n.hasParam(min_angle_key) || !n.hasParam(max_angle_key) || !n.hasParam(max_speed_key))
	  ROS_ERROR("Could not find one or all of the following parameters:\n %s\n %s\n %s", min_angle_key.c_str(), max_angle_key.c_str(), max_speed_key.c_str());
	
	n.getParam(min_angle_key, _joint_lower_bounds[i]);
	n.getParam(max_angle_key, _joint_upper_bounds[i]);
	n.getParam(max_speed_key, _joint_max_speeds[i]);
	
	// convert angles to radiant
	_joint_lower_bounds[i] = normalize_angle_rad( deg_to_rad( _joint_lower_bounds[i] ) ); // normalize to (-pi, pi]
	_joint_upper_bounds[i] = normalize_angle_rad( deg_to_rad( _joint_upper_bounds[i] ) ); // normalize to (-pi, pi]
	_joint_max_speeds[i] = deg_to_rad( _joint_max_speeds[i] );
  }     
  _initialized = true;
}

void PhantomXControl::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  for (int i=0; i<msg->name.size(); ++i)
  {
    int joint_idx = _map_joint_to_index[msg->name[i]];
    _joint_angles[joint_idx] = msg->position[i];
  }
}

void PhantomXControl::getJointAngles(Eigen::Ref<JointVector> values_out)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  values_out = _joint_angles;
}

void PhantomXControl::getJointAngles(std::vector<double>& values_out)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  values_out.assign(_joint_angles.data(), _joint_angles.data()+_joint_angles.rows());
}

void PhantomXControl::getJointVelocities(Eigen::Ref<JointVector> velocities_out)
{
  std::lock_guard<std::mutex> lock(_joints_mutex);
  velocities_out = _joint_velocities;
}

double PhantomXControl::getSlowestMaxSpeed() const
{
  return _joint_max_speeds.minCoeff();
}

  
void PhantomXControl::setJointsDefault(const ros::Duration& duration, bool blocking)
{
  setJoints(JointVector::Zero(), duration, blocking);
}
  
void PhantomXControl::setJointsDefault(double speed, bool blocking)
{
  setJoints(JointVector::Zero(), speed, blocking);
}  
  
void PhantomXControl::setJoints(const Eigen::Ref< const JointVector>& values, const ros::Duration& duration, bool blocking)
{
  ROS_ASSERT_MSG(_arm_action, "PhantomXControl: class not initialized, call initialize().");
  
  JointVector current_states;  
  getJointAngles(current_states); // we need to do a copy here, rather accessing '_joint_angles' directly,
				  // since receiving new states is multi threaded.
  
  // get maximum absolute angle difference
  double max_diff = (values - current_states).cwiseAbs().maxCoeff();

  // check and bound velocity
  // get maximum allowed duration (w.r.t. min of all max joint speeds)
  double duration_bounded = lower_bound(max_diff / getSlowestMaxSpeed(), duration.toSec()); // phi = omega*t -> t = phi/omega
  ROS_WARN_COND(duration_bounded>duration.toSec(), "PhantomXControl::setJoints(): desired speed is too high in order to drive all joints at this speed. Bounding...");
  
  // create trajectory to the single goal
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = _joint_names_arm;
  goal.trajectory.header.stamp = ros::Time::now();
  
  goal.trajectory.points.resize(1);
  goal.trajectory.points.front().time_from_start = ros::Duration(duration_bounded);
  goal.trajectory.points.front().positions.assign(values.data(), values.data()+values.rows());

  if (blocking)
    _arm_action->sendGoalAndWait(goal, ros::Duration(20));
  else
    _arm_action->sendGoal(goal);
}

void PhantomXControl::setJoints(const std::vector<double>& values, const ros::Duration& duration, bool blocking)
{
  Eigen::Map<const JointVector> values_map(values.data());
  setJoints(values_map, duration, blocking);
}

void PhantomXControl::setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool blocking)
{
  // check joint angle limits
  if (isExceedingJointLimits(values))
  {
    ROS_WARN_STREAM("PhantomXControl::setJoints(): cannot set new joint values since they exceed joint limits.\ndesired angles: ["
		    << values.transpose() << "]\nlower bounds: [" 
		    << _joint_lower_bounds.transpose() << "]\nupper bounds: [" << _joint_upper_bounds.transpose() << "]");
    return;
  }
  
  
  JointVector current_states;  
  getJointAngles(current_states); // we need to do a copy here, rather accessing '_joint_angles' directly,
				  // since receiving new states is multi threaded.
  
  // get maximum absolute angle difference (TODO: do we need to normalize the angles before taking abs()?)
  double max_diff = (values - current_states).cwiseAbs().maxCoeff();

  // get time corresponding to the distace max_diff and the given speed value
  // assume a constant velocity: phi=omega*t
  double duration = max_diff/speed;
  if (duration<0)
  {
    ROS_ERROR("PhantomXControl::setJoints(): obtained an invalid (negative) duration. Cannot set new joint values.");
    return;
  }
  setJoints(values,ros::Duration(duration),blocking);
}
  
void PhantomXControl::setJoints(const std::vector<double>& values, double speed, bool blocking)
{
  Eigen::Map<const JointVector> values_map(values.data());
  setJoints(values_map, speed, blocking);
}  
  
  
void PhantomXControl::commandJointVel(const Eigen::Ref<const JointVector>& velocities, bool blocking)
{
  
}
  
  
  
void PhantomXControl::setJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory, bool blocking)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  
  setJointTrajectory(goal, blocking);
}

void PhantomXControl::setJointTrajectory(const control_msgs::FollowJointTrajectoryGoal& trajectory, bool blocking)
{
  ROS_ASSERT_MSG(_arm_action && _initialized, "PhantomXControl: class not initialized, call initialize().");
    
  if (blocking)
    _arm_action->sendGoalAndWait(trajectory, ros::Duration(20));
  else
    _arm_action->sendGoal(trajectory);
}


bool PhantomXControl::isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values)
{
  return (joint_values.array()<_joint_lower_bounds.array()).any() || (joint_values.array()>_joint_upper_bounds.array()).any();
}
  
  
  
} // end namespace phantomx