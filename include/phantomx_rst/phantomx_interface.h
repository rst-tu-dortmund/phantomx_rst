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

#ifndef PHANTOMX_INTERFACE_H_
#define PHANTOMX_INTERFACE_H_

// stl
#include <memory>
#include <mutex>

// ros stuff
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>

// eigen 
#include <Eigen/Core>

// own stuff
#include <phantomx_rst/misc.h>




namespace phantomx
{
  
//! Typedef for a joint vector q=[q1,q2,q3,q4]^T  
using JointVector = Eigen::Matrix<double,4,1>;

//! Struct for storing joint specific information
// struct JointDetails
// {
//   double min_angle = 0;
//   double max_angle = 0;
//   double max_speed = 0;
// };


/**
 * @class PhantomXControl
 * @brief This class provides a common interface to communicate with the PhantomX Pincher Arm robot.
 * 
 * This class does not implement a low-level hardware API in order to send actual robot commands to the 
 * Arbotix board on the robot. Rather the arbotix-ros package is utilized for this job.
 * The PhantomXControl class provides a slighly more high-level API to set velocity, position and trajectory following commands.
 * It is not expected to replace MoveIt (which is also compatible with the PhantomX), but this package is intended for a
 * much simpler and guided usage on the control level suited for education and research.
 */
class PhantomXControl
{
public:
  
  /**
   * @brief Construct the class
   */
  PhantomXControl();
  
  /**
   * @brief Destruct the class
   */
  virtual ~PhantomXControl();
  
  /**
  * @brief Initializes the class
  */
  void initialize(); 
  
  /** @name Receive robot / joint state information */
  //@{
  
  /**
   * @brief Reveive joint names according to the ordering in the JointVector
   * @return read-only reference to the vector containing joint name strings
   */
  const std::vector<std::string>& getJointNames() const {return _joint_names_arm;}
  
  /**
   * @brief Get current joint angles 
   * @param[out] values_out Eigen Matrix/Vector type q=[q1,q2,q3,q4]^T (doubles) which the joint angles are written to.
   */
  void getJointAngles(Eigen::Ref<JointVector> values_out);
  
  /**
   * @brief Get current joint angles 
   * @param[out] values_out std::vector< double > which the joint angles are written to.
   */
  void getJointAngles(std::vector<double>& values_out);
  
  /**
   * @brief Get current joint velocities 
   * @param[out] values_out Eigen Matrix/Vector type q=[q1,q2,q3,q4]^T (doubles) which the joint velocities are written to.
   */
  void getJointVelocities(Eigen::Ref<JointVector> velocities_out);

  /**
   * @brief Get the slowest max speed of the set of all joints
   * @todo this could be calculated once and stored as class property
   * @return min( qdot1_max, qdot2_max, qdot3_max, qdot4_max )
   */  
  double getSlowestMaxSpeed() const;
  
  //@}
  
  
  /** @name Command joint angles */
  //@{
    
  /**
   * @brief Set joints to the default position q=[0,0,0,0]^T
   * @param duration duration for the transition to the new joint state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJointsDefault(const ros::Duration& duration=ros::Duration(5), bool blocking=true);
  
  /**
   * @brief Set joints to the default position q=[0,0,0,0]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the default position.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */ 
  void setJointsDefault(double speed, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param duration duration for the transition to the new joint state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, const ros::Duration& duration=ros::Duration(5), bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param duration duration for the transition to the new joint state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const std::vector<double>& values, const ros::Duration& duration=ros::Duration(5), bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const std::vector<double>& values, double speed, bool blocking=true);
  
  //@}
  
  /** @name Command joint velocities */
  //@{
  
  void commandJointVel(const Eigen::Ref<const JointVector>& velocities, bool blocking=true);
  
  //@}
  
  
  /** @name Follow joint trajectory */
  //@{
  
  /**
   * @brief Follow a given joint trajectory.
   * 
   * The trajectory should contain a sequence of positions. For each position the joint angles needs to be
   * matched with the joint names. You can obtain joint names from the getJointAngles() method.
   * @code
   *    // Example trajectory:
   * 	trajectory_msgs::JointTrajectory trajectory;
   *	trajectory.header.stamp = ros::Time::now(); // we want to execute the trajectry now
   *	trajectory.joint_names = robot.getJointNames(); // robot denotes a PhantomXControl object instance.
   *	trajectory.points.resize(2); // we want two goal states
   *	// intermediate goal:
   *	robot.getJointAngles( trajectory.points[0].positions ); // set intermediate goal to current values
   *	trajectory.points[0].positions[0] = -M_PI/3; // set first joint to -pi/3
   *	trajectory.points[0].time_from_start = ros::Duration(4); // 2 seconds for the transition
   *	// final goal:
   *	trajectory.points[1].positions.resize(4, 0); // initialize all 4 joints to 0
   *	trajectory.points[1].positions[2] = M_PI/2; // set joint 2 to pi/2
   *	trajectory.points[1].time_from_start = ros::Duration(5); // 3 seconds for the transition
   * @endcode
   * 
   * @param trajectory JointTrajectory message containing the new positions
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory, bool blocking=true);
  
  /**
   * @brief Follow a given joint trajectory
   * 
   * See setJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory, bool blocking) for details on
   * how to specify the underlying trajectory.
   * @param trajectory Trajectory given in FollowJointTrajectoryGoal format
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing
   */
  void setJointTrajectory(const control_msgs::FollowJointTrajectoryGoal& trajectory, bool blocking=true);
  
  //@}
  
  
   /** @name Utility methods */
  //@{ 

  /**
   * @brief Check if a given joint vector exceeds joint limits
   * @param joint_values vector of joint values q=[q1,q2,q3,q4]^T
   * @return \c true if the joint vector exceeds limits, \c false otherwise
   */
  bool isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values);
  
  //@}
  
private:
    
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  
  
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> _arm_action; //!< Action client for trajectory following
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> _gripper_action; //!< Action client for gripper actions
  
  ros::Subscriber _joints_sub;
  ros::CallbackQueue* _joints_sub_queue = nullptr; // seems to be deleted by ros
  std::unique_ptr<ros::AsyncSpinner> _joints_sub_spinner;
  
  std::mutex _joints_mutex;
  JointVector _joint_angles = JointVector::Zero();
  JointVector _joint_velocities = JointVector::Zero();
  
  JointVector _joint_lower_bounds;
  JointVector _joint_upper_bounds;
  JointVector _joint_max_speeds;
  
  std::map<std::string, int> _map_joint_to_index;
  
  std::vector<std::string> _joint_names_arm; //!< Store names for all joints of the arm
//   std::vector<JointDetails> _joint_details; //!< Store information of all joints of the arm (angle limits and max speed)
  
  bool _initialized = false;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace phantomx


#endif /* PHANTOMX_INTERFACE_H_ */
