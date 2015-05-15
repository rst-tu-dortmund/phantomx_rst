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
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// own stuff
#include <phantomx_rst/types.h>
#include <phantomx_rst/misc.h>
#include <phantomx_rst/kinematics.h>



namespace phantomx
{
  

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
  
  /** @name Receive joint state information */
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
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, const ros::Duration& duration=ros::Duration(5), bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param duration duration for the transition to the new joint state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @remarks This overload accepts initializer-lists: <code> setJoints({0.3, 0, 0, 0}) </code>
   */
  void setJoints(const std::vector<double>& values, const ros::Duration& duration=ros::Duration(5), bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, double speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed speed for the transition w.r.t. the joint with the highest deviation from the final state.
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @remarks This overload accepts initializer-lists: <code> setJoints({0.3, 0, 0, 0}) </code>
   */
  void setJoints(const std::vector<double>& values, double speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles by commanding individual joint velocities
   * @param values vector of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed vector containing joint velocities qdot = [omega1, omega2, omega3, omega4]^T
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const Eigen::Ref<const JointVector>& values, const Eigen::Ref<const JointVector>& speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles by commanding individual joint velocities
   * @param values std::vector< double > of new joint angles q=[q1,q2,q3,q4]^T
   * @param speed std::vector< double > containing joint velocities qdot = [omega1, omega2, omega3, omega4]^T
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   * @remarks This overload accepts initializer-lists: <code> setJoints({0.3, 0, 0, 0}, {0.1, 0, 0, 0}) </code>
   */
  void setJoints(const std::vector<double>& values, const std::vector<double>& speed, bool relative=false, bool blocking=true);
  
  /**
   * @brief Command new joint angles
   * 
   * You can either specify a synchronous transition by setting \c joint_state.time_from_start to a given duration,
   * or you can set each joint velocity individual by providing \c joint_state.velocities.
   * @param joint_state trajectory_msgs::JointTrajectoryPoint type
   * @param relative if \c true, new joint states are relative to the previous one, otherwise absolute
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJoints(const trajectory_msgs::JointTrajectoryPoint& joint_state, bool relative=false, bool blocking=true);
  
  //@}
  
  /** @name Command joint velocities */
  //@{
  
  /**
   * @brief Command robot by specifying joint velocities.
   * 
   * Be careful, this call is a non-blocking call and the robot will
   * hold the specified velocities until joint limits are exceeded or
   * a new command is sent.
   * @param velocities vector of desired joint velocities
   */
  void setJointVel(const Eigen::Ref<const JointVector>& velocities);
  
  /**
   * @brief Command robot by specifying joint velocities.
   * 
   * Be careful, this call is a non-blocking call and the robot will
   * hold the specified velocities until joint limits are exceeded or
   * a new command is sent.
   * @remarks This overload accepts initializer-lists: <code> setJointVel({0.1, 0.1, 0, 0}) </code>
   * @param velocities std::vector< double > of desired joint velocities
   */
  void setJointVel(const std::vector<double>& velocities);
  
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
   * @param trajectory JointTrajectory message containing the new positions (non-const, since velocity could be limited: warnings appear)
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing 
   */
  void setJointTrajectory(trajectory_msgs::JointTrajectory& trajectory, bool blocking=true);
  
  /**
   * @brief Follow a given joint trajectory
   * 
   * See setJointTrajectory(const trajectory_msgs::JointTrajectory& trajectory, bool blocking) for details on
   * how to specify the underlying trajectory.
   * @param trajectory Trajectory given in FollowJointTrajectoryGoal format (non-const, since velocity could be limited: warnings appear)
   * @param blocking if \c true, wait until the goal is reached or the timeout is exceeded before continuing
   */
  void setJointTrajectory(control_msgs::FollowJointTrajectoryGoal& trajectory, bool blocking=true);
  
  //@}
  
  
  /** @name Kinematics and Differential Kinematics */
  //@{
  
  /**
   * @brief Get the transformation matrix to the gripper(TCP) expressed in the robot base System
   * @param[out] base_T_gripper Eigen::Affine3d transformation type
   */
  void getEndeffectorState(Eigen::Affine3d& base_T_gripper);
  
  /**
   * @brief Get the transformation matrix to the gripper(TCP) expressed in the robot base System
   * @param[out] base_T_gripper tf transformation type
   */
  void getEndeffectorState(tf::StampedTransform& base_T_gripper);
  
  /**
   * @brief Compute the geometric jacobian of the endeffector/gripper velocity w.r.t. to the base frame.
   * 
   * The geometric jacobian denotes the linear relationship between the endeffector velocity and the joint angle 
   * velocities w.r.t. to the robot base frame: v = J*qdot.
   * Each row corresponds to the joint velocities and each row correspond to the endeffector velocity.
   * The resulting matrix is therefore 6x4.
   * The rotational component (lower 3x4 submatrix) expresses the rotational component of the end effector velocity
   * in terms of angular velocity (angle and axis representation). 
   * @see KinematicModel::computeJacobian
   * @remarks This is not the analytic robot jacobian!
   * @param[out] jacobian The 6x4 jacobian will be written into this matrix.
   */
  void getJacobian(RobotJacobian& jacobian);
  
  /**
   * @brief Access the kinematics model of the robot
   * 
   * Use the kinematics model in order to calculate/simulate forward/differential/inverse kinematics
   * for arbitrary joint states
   * @return KinematicModel object (read-only)
   */
  const KinematicModel& kinematics() const {return _kinematics;}
  
  
  //! A small test script that drives into a predefined set of joint configuratoins in order to test the underlying KinematicModel
  bool testKinematicModel();
  //! Drives to a joint configuration and verify the forward kinematics of the underlying KinematicModel
  bool testKinematicModel(const Eigen::Ref<const JointVector>& joint_values);
  
  //@}
  
  
   /** @name Utility methods */
  //@{ 

  /**
   * @brief Check if a given joint vector exceeds joint limits
   * @param joint_values vector of joint values q=[q1,q2,q3,q4]^T
   * @return \c true if the joint vector exceeds limits, \c false otherwise
   */
  bool isExceedingJointLimits(const Eigen::Ref<const JointVector>& joint_values);
    
  /**
   * @brief Stop any running transition.
   */
  void stopMoving();
  
  //@}
  
  
  
protected:
  
  /**
   * @brief Check and adapt trajectory in order to satisfy joint restrictions.
   * @param trajectory parameter to be checked and modified
   * @return \c true if trajectory is feasible (some velocity adjustments could take place, check warnings),
   *	     \c false if joint angle limits are exceeded.
   */
  bool verifyTrajectory(trajectory_msgs::JointTrajectory& trajectory);
  
  
private:
    
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  
  
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> _arm_action; //!< Action client for trajectory following
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> _gripper_action; //!< Action client for gripper actions
  
  KinematicModel _kinematics;
  
  ros::Subscriber _joints_sub;
  ros::CallbackQueue* _joints_sub_queue = nullptr; // seems to be deleted by ros
  std::unique_ptr<ros::AsyncSpinner> _joints_sub_spinner;
  bool _joint_values_received = false;
  tf::TransformListener _tf;
  
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
