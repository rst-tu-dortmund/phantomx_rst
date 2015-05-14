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


namespace phantomx
{

Eigen::Affine3d KinematicModel::computeForwardKinematics(const Eigen::Ref<const JointVector>& joint_values) const
{
  Eigen::Affine3d transform = _base_T_arm;
  
  // Everything here is hardcoded according to the coordinate systems chosen in the robot urdf provided by the
  // turtlebot_arm package. Unfortunately, they do not use the DH convention.
  Eigen::Vector3d unit_y = Eigen::Vector3d::UnitY();
  
  // first joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[0], unit_y ) );
    
  // first joint to second joint
  transform = transform * _j1_T_j2;
  
  // second joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[1], unit_y ) );
  
  // second joint to third joint
  transform = transform * _j2_T_j3;
  
  // third joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[2], unit_y ) );
  
  // third joint to fourth joint
  transform = transform * _j3_T_j4;
  
  // fourth joint (rotate around y)
  transform.rotate( Eigen::AngleAxisd( joint_values[3], unit_y ) );  
  
  // fourth joint to gripper
  transform = transform * _arm_T_gripper;
  
  return transform;
}
  
  
} // end namespace phantomx