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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

// ros stuff
#include <ros/ros.h>

// own stuff
#include <phantomx_rst/types.h>
#include <phantomx_rst/misc.h>




namespace phantomx
{
  


/**
 * @class KinematicsModel
 * @brief This class provides methods regarding the kinematics of the 4-DOF PhantomX pincher robot arm.
 * 
 * This class allows the user to e.g. simulate end effector poses for varying joint states, inverse kinematics
 * and differential kinematics. It is currently based on the Denavit-Hartenberg parameters of the robot.
 * Unfortunately, we cannot apply DH parameters since the turtlebot_arm package does not define the coordinate systems
 * in the robot URDF according to the convention. E.g. all rotations are around the y-axis.
 * 
 * If you want to simulate the kinematics model in Matlab using Peter Corke's Robotics Toolbox, use the following model:
 * @code
 *      l1 = Link('revolute','d',0.065,'a',0,'alpha',-pi/2);
 *      l2 = Link('revolute','d',0,'a',0.1015,'alpha',0,'offset',-pi/2);
 *      l3 = Link('revolute','d',0,'a',0.1015,'alpha',0);
 *      l4 = Link('revolute','d',0,'a',0.0880,'alpha',0);
 *      arm = SerialLink([l1,l2,l3,l4]);
 *      arm.tool = troty(-pi/2) * trotx(pi);
 * @endcode
 * But only the relation between the arm_base_link and gripper frame are identically. The individual joint 
 * coordinate systems differ from each other (since the robotics toolbox relies on DH paramters)
 * 
 * @todo Make templated for arbitrary robots
 * @todo Extract trasnformations from URDF/Mesh files of the robot.
 * @remarks We restrict ourselves to revolute joints only.
 */
class KinematicModel
{
public:
  
  /**
   * @brief Construct the class
   */
  KinematicModel() {};
  
  /**
   * @brief Destruct the class
   */
  virtual ~KinematicModel() {};
  
  //! set the coordinate transformation from the base coordinate system to the system of joint 1 (for <b>q=0</b>)
  void setBaseToJoint1Transform(const Eigen::Affine3d& base_T_arm) {_base_T_arm = base_T_arm;};
  //! set the coordinate transformation from joint 1 to joint 2 for <b>q=0</b> (joint 1 rotates around y)
  void setJoint1ToJoint2Transform(const Eigen::Affine3d& j1_T_j2) {_j1_T_j2 = j1_T_j2;};
  //! set the coordinate transformation from joint 2 to joint 3 for <b>q=0</b> (joint 2 rotates around y)
  void setJoint2ToJoint3Transform(const Eigen::Affine3d& j2_T_j3) {_j2_T_j3 = j2_T_j3;};
  //! set the coordinate transformation from joint 3 to joint 4 for <b>q=0</b> (joint 3 rotates around y)
  void setJoint3ToJoint4Transform(const Eigen::Affine3d& j3_T_j4) {_j3_T_j4 = j3_T_j4;};
  //! set the coordinate transformation from joint 4 to the gripper (TCP) coordiante system for <b>q=0</b> (joint 3 rotates around y)
  void setJoint4ToGripperTransform(const Eigen::Affine3d& arm_T_gripper) {_arm_T_gripper = arm_T_gripper;};
  
  /**
   * @brief Compute the forward kinematics according to the specified joint angles.
   * @param joint_values Vector of joint values q=[q1,q2,q3,q4]^T
   * @param up_to_index Specify final frame: \c 0 -> joint1 frame, \c 1 -> joint2 frame, \c 2 -> joint3 frame
   *                                         \c 3 -> joint 4 frame, \c 4 -> gripper / TCP frame
   *                    Be careful if you use this for other purposes than computing the jacobian, since
   *                    the rotation of a joint angle defines rotation around y to the subsequent frame instead of the current one:
   *                    E.g. joint1 frame is independent of the angle of joint1. This does not correspond to the actual
   *                    transformation provided by tf!
   * @return Transformation from the base coordinate system to the gripper (tool-center-point) coordinate system
   */
  Eigen::Affine3d computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values, int up_to_index=4) const;
  
  /**
   * @brief Compute the geometric jacobian of the endeffector/gripper velocity w.r.t. to the base frame.
   * 
   * The geometric jacobian denotes the linear relationship between the endeffector velocity and the joint angle 
   * velocities w.r.t. to the robot base frame: v = J*qdot.
   * Each row corresponds to the joint velocities and each row correspond to the endeffector velocity.
   * The resulting matrix is therefore 6x4.
   * The rotational component (lower 3x4 submatrix) expresses the rotational component of the end effector velocity
   * in terms of angular velocity (angle and axis representation). 
   * @remarks This is not the analytic robot jacobian!
   * @param joint_values Joint configuration q=[q1,q2,q3,q4] in which the jacobian should be computed
   * @param[out] jacobian The 6x4 jacobian will be written into this matrix.
   */
  void computeJacobian(const Eigen::Ref<const JointVector>& joint_values, RobotJacobian& jacobian) const;
  
  void inverseKinematics(const Eigen::Ref<const JointVector>& joint_values);

 
  
private:
    
  Eigen::Affine3d _base_T_arm = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j1_T_j2 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j2_T_j3 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _j3_T_j4 = Eigen::Affine3d::Identity();
  Eigen::Affine3d _arm_T_gripper = Eigen::Affine3d::Identity();
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace phantomx


#endif /* KINEMATICS_H_ */
