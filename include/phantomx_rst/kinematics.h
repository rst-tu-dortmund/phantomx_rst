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
  
    
  void setBaseToArmTransform(const Eigen::Affine3d& base_T_arm) {_base_T_arm = base_T_arm;};
  void setJoint1ToJoint2Transform(const Eigen::Affine3d& j1_T_j2) {_j1_T_j2 = j1_T_j2;};
  void setJoint2ToJoint3Transform(const Eigen::Affine3d& j2_T_j3) {_j2_T_j3 = j2_T_j3;};
  void setJoint3ToJoint4Transform(const Eigen::Affine3d& j3_T_j4) {_j3_T_j4 = j3_T_j4;};
  void setArmToGripperTransform(const Eigen::Affine3d& arm_T_gripper) {_arm_T_gripper = arm_T_gripper;};
  
  Eigen::Affine3d computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values) const;
  
protected:
  

  
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
