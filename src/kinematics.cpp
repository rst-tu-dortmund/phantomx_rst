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
  Eigen::Affine3d transform = _base_T_j1;
  
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
  transform = transform * _j4_T_gripper;
  
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
  

bool KinematicModel::computeInverseKinematics(const Eigen::Affine3d& desired_pose, Eigen::Ref<JointVector> joint_values, bool force_yaw) const
{
  //TODO
  // VALIDATE POSE (MAX DISTANCE)

  // Geometric approach:
  // Start with the angle around z (in x-y plane, since we have only a single joint for that DOF).
  // Transform the desired pose from the base frame into the frame of the first joint
  Eigen::Affine3d j1_T_pose = _j1_T_base * desired_pose; // _j1_T_base does not include the angle of the first joint by definition, therefore it is q1=0.
        
  RpyVector rpy = rotMatToRpy(j1_T_pose.linear());
  // in this frame, the pitch value corresponds to the desired rotation (since we rotate around y).
  double q1_cand1 = rpy.coeffRef(1);  
  
  // check translation vector as well
  double q1_cand2 = atan2( j1_T_pose.translation().x(), j1_T_pose.translation().z() ); // because of the coordinate system of j1, x() replaces the old y-axis.
  double q1_cand3 = normalize_angle_rad(q1_cand2+M_PI);
  // q1_cand1 should be equal to one of the others, since we have only a single degree of freedom for rotation in the x-y axis
  
  if (fabs(q1_cand1-q1_cand2) < 1e-2 || fabs(q1_cand1-q1_cand3)<1e-2)
  {
      // First test if we can reach the desired yaw angle

      // we must rotate j1 first by the angle q1 that we have already computed:
      joint_values.coeffRef(0) = q1_cand1;
      Eigen::Affine3d j2_T_pose = _j2_T_j1 * Eigen::AngleAxisd( -joint_values.coeffRef(0), Eigen::Vector3d::UnitY() ) * j1_T_pose;
      bool success = computeIk3LinkPlanarElbowUpAndDown(j2_T_pose, joint_values.bottomRows(3));
      if (success)
	return true;
      else if (!force_yaw)
      {
	ROS_WARN_STREAM("No feasible solution found to satisfy the desired yaw angle. Trying to find the IK to the same position but with a different yaw...");	
	joint_values.coeffRef(0) = normalize_angle_rad(q1_cand1 + M_PI);
	j2_T_pose = _j2_T_j1 * Eigen::AngleAxisd( -joint_values.coeffRef(0), Eigen::Vector3d::UnitY() ) * j1_T_pose;
	if (computeIk3LinkPlanarElbowUpAndDown(j2_T_pose, joint_values.bottomRows(3)))
	  return true;
      }
  }
  else
  {
    ROS_WARN_ONCE("InverseKinematics: The specified yaw angle should be equal to the orientation of the direction vector (pi may be added), since the arm has only a single degree of freedom for this motion part (this warning is displayed once)."); // yaw angle in the base coordinate system
    if (!force_yaw)
    {
      ROS_WARN("Considering the translation and roll only, ignoring the yaw-angle.");
      if (fabs(q1_cand2-q1_cand1) < fabs(q1_cand3-q1_cand1))
      {
	// test cand2 first
	joint_values.coeffRef(0) = q1_cand2;
	Eigen::Affine3d j2_T_pose = _j2_T_j1 * Eigen::AngleAxisd( -joint_values.coeffRef(0), Eigen::Vector3d::UnitY() ) * j1_T_pose;
	bool success = computeIk3LinkPlanarElbowUpAndDown(j2_T_pose, joint_values.bottomRows(3));
	if (success)
	  return true;
	else
	{	
	  joint_values.coeffRef(0) = q1_cand3;
	  j2_T_pose = _j2_T_j1 * Eigen::AngleAxisd( -joint_values.coeffRef(0), Eigen::Vector3d::UnitY() ) * j1_T_pose;
	  if (computeIk3LinkPlanarElbowUpAndDown(j2_T_pose, joint_values.bottomRows(3)))
	    return true;
	}
      }
      else
      {
	// test cand3 first
	joint_values.coeffRef(0) = q1_cand3;
	Eigen::Affine3d j2_T_pose = _j2_T_j1 * Eigen::AngleAxisd( -joint_values.coeffRef(0), Eigen::Vector3d::UnitY() ) * j1_T_pose;
	bool success = computeIk3LinkPlanarElbowUpAndDown(j2_T_pose, joint_values.bottomRows(3));
	if (success)
	  return true;
	else
	{	
	  joint_values.coeffRef(0) = q1_cand2;
	  j2_T_pose = _j2_T_j1 * Eigen::AngleAxisd( -joint_values.coeffRef(0), Eigen::Vector3d::UnitY() ) * j1_T_pose;
	  if (computeIk3LinkPlanarElbowUpAndDown(j2_T_pose, joint_values.bottomRows(3)))
	    return true;
	}
      }
      
    }
  }
  
  ROS_ERROR_STREAM("InverseKinematics: No solution found");
  return false;
}

bool KinematicModel::computeIk3LinkPlanar(const Eigen::Affine3d& j2_T_pose, Eigen::Ref<Eigen::Vector3d> values, bool elbow_up) const
{
  // we first compute the two link planar arm solution w.r.t. to the position of the last joint (denoted as point "w")
  // In order to obtain the position of the last joint "w", we translate the desired pose by the link length along the negative "z" axis w.r.t. to the system of joint1.
  // WARNING: z2,z3,z4 axes must be aligned with the links in the default joint configuration (since we formulate the IK for according to the URDF from the turtlebot_arm package)!
  //	      for joint1: y1 is aligned with the link instead of z1 (for q1=0)
  // Joint angles are negative and they are identical zero at the z-axis!!!!!!!!!!!!!!!!
  // Nomenclature of the "underlying" three link planar arm:
  // a1: length of link1
  // a2: length of link2
  // a3: length of link3
  double a1 = fabs(_j2_T_j3.translation().z());
  double a2 = fabs(_j3_T_j4.translation().z());
  double a3 = fabs(_j4_T_gripper.translation().z());
  
  // now transform the point (0,0,-a3) w.r.t. the gripper system to the j2 system
  Eigen::Vector3d w = j2_T_pose * Eigen::Vector3d(0,0,-a3); // we can ignore the y compontent since we operate planar only from now on

  double w_sq_length_2d = w.z()*w.z() + w.x()*w.x();
  
//   ROS_INFO_STREAM("a1=" << a1 << " a2=" << a2 << " a3=" << a3);
//   ROS_INFO_STREAM("\nw: " << w.transpose() << " w_length: " << sqrt(w_sq_length_2d) << "\nphi: " << phi);

  double c2 = ( w_sq_length_2d - a1*a1 - a2*a2 ) / (2*a1*a2);
  ROS_ASSERT_MSG(c2!=0, " TODO: c2==0");
  
  // c2 must be within the interval -1 <= c2 <= 1
  // sometimes we have numerical issues:
  if ( fabs(c2-1) < 1e-2 )
    c2 = 1;
  if ( fabs(c2+1) < 1e-2 )
    c2 = -1;
  
  if (!isInsideInterval(-1.0,c2,1.0))
  {
    ROS_ERROR("Inverse Kinematics: No solution found.");
    return false;
  }
  
  double s2 = sqrt(1-c2*c2);
  
  double q2;
  if (elbow_up)
    q2 = -std::atan2( -s2, c2 ); 
  else
    q2 = -std::atan2( s2, c2 ); // up and down configurations are swapped in contrast to many example books,
						// since our angle is oriented clock-wise (in the plane) (therefore the minus sign)  
  values.coeffRef(1) = q2;
  
  // now compute the value of joint 2
  double aux = a1+a2*cos(-values.coeffRef(1));
  if ( fabs(w.x())<1e-2 && fabs(aux)<1e-2) // if w.x()==0  or aux==0, link a1 and a2 are anti-parallel and colinear, which leads to a self-collision
  {
    ROS_ERROR("inverseKinematics: Cannot determine joint angle q2 for the desired pose.");
    return false;
  }
 values.coeffRef(0) = normalize_angle_rad(M_PI/2 - std::atan2(w.z(),w.x()) + std::atan2(a2*sin(-values.coeffRef(1)),aux));  // We must add pi/2, since our default position for q2=0 is upwards!! 

    // determine desired final orientation of the gripper in the joint 2 frame
//   double phi = -1*std::atan2(-j2_T_pose.linear().coeffRef(0,2), j2_T_pose.linear().coeffRef(2,2));
  RpyVector rpy = rotMatToRpy(j2_T_pose.linear());
  double phi = rpy.coeffRef(0);  
  
  // it is phi = q2+q3+q4
  values.coeffRef(2) = normalize_angle_rad(phi - values.coeffRef(1) - values.coeffRef(0));
  
  return true;
}

bool KinematicModel::computeIk3LinkPlanarElbowUpAndDown(const Eigen::Affine3d& j2_T_pose, Eigen::Ref<Eigen::Vector3d> values) const
{
  Eigen::Vector3d q_elbowup, q_elbowdown;
  bool elbow_up_succ = computeIk3LinkPlanar(j2_T_pose, q_elbowup, true);
  bool elbow_down_succ = computeIk3LinkPlanar(j2_T_pose, q_elbowdown, false);
  elbow_up_succ = elbow_up_succ && isInsideInterval(_joint_lower_bounds.bottomRows(3), q_elbowup, _joint_upper_bounds.bottomRows(3));
  elbow_down_succ = elbow_down_succ && isInsideInterval(_joint_lower_bounds.bottomRows(3), q_elbowdown, _joint_upper_bounds.bottomRows(3));
  
  if (elbow_up_succ && elbow_down_succ)
  {
    // select that one that is nearer to our current joint angles
    if ( (q_elbowup-values).norm() < (q_elbowdown-values).norm())
    {
      values = q_elbowup;
      return true;
    }
    else
    {
      values = q_elbowdown;
      return true;
    }
  }
  else if (elbow_up_succ)
  {
    values = q_elbowup;
    return true;
  }
  else if (elbow_down_succ)
  {
    values = q_elbowdown;
    return true;
  }
  return false;
}
  
  
} // end namespace phantomx