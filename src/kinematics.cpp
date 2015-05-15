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

#include <phantomx_rst/kinematics.h>

#include <unsupported/Eigen/NonLinearOptimization>

namespace phantomx
{

    
Eigen::Affine3d KinematicModel::computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values, int up_to_index) const
{
  Eigen::Affine3d transform = _base_T_arm;
  
  if (up_to_index==0) // base to joint 1 only
      return transform;
  
  // Everything here is hardcoded according to the coordinate systems chosen in the robot urdf provided by the
  // turtlebot_arm package. Unfortunately, they do not use the DH convention.
  Eigen::Vector3d unit_y = Eigen::Vector3d::UnitY();
  
  // first joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[0], unit_y ) );
    
  // first joint to second joint
  transform = transform * _j1_T_j2;
  
  if (up_to_index==1) // base to joint 2 only
    return transform;
  
  // second joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[1], unit_y ) );
  
  // second joint to third joint
  transform = transform * _j2_T_j3;
  
  if (up_to_index==2) // base to joint 3 only
    return transform;
  
  // third joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[2], unit_y ) );
  
  // third joint to fourth joint
  transform = transform * _j3_T_j4;
  
  if (up_to_index==3) // base to joint 4 only
    return transform;
  
  // fourth joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[3], unit_y ) );  
  
  // fourth joint to gripper
  transform = transform * _arm_T_gripper;
  
  return transform;
}
  


  
void KinematicModel::computeJacobian(const Eigen::Ref<const JointVector>& joint_values, RobotJacobian& jacobian) const
{
    // Consider only revolute joints
    // translational part: z_{i-1} x p{i-1,E}  (p denotes the distance vector from joint i-1 to the endeffector)
    // rotational part: z_{i-1}
    // Since our model specifies the y-axis to be the axis of rotation, we have to substitute z_{i-1} by y_{i-1}.
    // Therefore our second column of the rotation matrix represents the unit vector of the axis of rotation in our base frame.
    
    Eigen::Affine3d tcp_frame = computeForwardKinematics(joint_values,4);
        
    // joint 1
    Eigen::Affine3d prev_frame = computeForwardKinematics(joint_values,0);
    jacobian.block(0,0,3,1) =  prev_frame.rotation().col(1).cross( tcp_frame.translation() - prev_frame.translation() ); // position part joint 1
    jacobian.block(3,0,3,1) =  prev_frame.rotation().col(1); // rotation part joint 1
    
    // joint 2
    prev_frame = computeForwardKinematics(joint_values,1);
    jacobian.block(0,1,3,1) =  prev_frame.rotation().col(1).cross( tcp_frame.translation() - prev_frame.translation() ); // position part joint 2
    jacobian.block(3,1,3,1) =  prev_frame.rotation().col(1); // rotation part joint 2
    
    // joint 3
    prev_frame = computeForwardKinematics(joint_values,2);
    jacobian.block(0,2,3,1) =  prev_frame.rotation().col(1).cross( tcp_frame.translation() - prev_frame.translation() ); // position part joint 3
    jacobian.block(3,2,3,1) =  prev_frame.rotation().col(1); // rotation part joint 3
    
    // joint 4
    prev_frame = computeForwardKinematics(joint_values,3);
    jacobian.block(0,3,3,1) =  prev_frame.rotation().col(1).cross( tcp_frame.translation() - prev_frame.translation() ); // position part joint 4
    jacobian.block(3,3,3,1) =  prev_frame.rotation().col(1); // rotation part joint 4
}  
  

void KinematicModel::inverseKinematics(const Eigen::Ref<const JointVector>& joint_values)
{

}

  
  
} // end namespace phantomx