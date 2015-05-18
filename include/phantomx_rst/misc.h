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

#ifndef MISC_H_
#define MISC_H_

#include <memory>
#include <math.h>
#include <type_traits>

#include <phantomx_rst/types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Geometry>

  

namespace phantomx
{
  
  
  /**
   * @brief Convert angle from degree to radiant
   * @param angle_deg angle given in degree
   * @return angle in radiant [0, 2pi]
   */
  inline double deg_to_rad(double angle_deg)
  {
    return angle_deg * 0.01745329251994329576; // pi/180
  }
  
  /**
   * @brief Convert angle from radiant to degree
   * @param angle_rad angle given in degree
   * @return angle in degree [0, 360]
   */
  inline double rad_to_deg(double angle_rad)
  {
    return angle_rad * 57.29577951308232087721; // 180/M_PI
  }
  
  /**
   * @brief Normalize angle in radiant to the interval (-pi,2]
   * @param angle angle given in radiant
   * @return angle in radiant (-pi, pi]
   */  
  inline double normalize_angle_rad(double angle)
  {
    if (angle >= -M_PI && angle < M_PI)
      return angle;
    
    double mu = floor(angle / (2 * M_PI));
    angle = angle - mu * 2 * M_PI;
    
    if (angle >= M_PI)
      angle -= 2*M_PI;
    
    if (angle < -M_PI)
      angle += 2*M_PI;
    
    return angle;
  }
  
  /**
   * @brief Normalize angle in degree to the interval [0, 360)
   * @param angle angle given in degree
   * @return angle in degree [0, 360)
   */  
  inline double normalize_angle_deg_360(double angle)
  {
    if (angle >= 0 && angle < 360)
      return angle;
        
    return fmod(angle, 360.);
  }
  
  /**
   * @brief Check if x is inside the interval [l, u]
   * @param l lower bound
   * @param x value to be checked
   * @param u upper bound
   * @tparam T numerical/scalar type
   * @return \c true if inside, otherwise \c false
   */
  template <typename T, typename std::enable_if<std::is_scalar<T>::value>::type* = nullptr>
  inline bool isInsideInterval(T l, T x, T u) 
  {
    if (x < l)
      return false;
    if (x > u)
      return false;
    return true;
  }
  
  /**
   * @brief Check if each component of vector x is inside the corresponding interval [vec_l, vec_u]
   * @param l lower bound vector
   * @param x vector to be checked
   * @param u upper bound vector
   * @return \c true if inside, otherwise \c false
   */
  inline bool isInsideInterval(const Eigen::Ref<const Eigen::VectorXd>& l, const Eigen::Ref<const Eigen::VectorXd>& x, const Eigen::Ref<const Eigen::VectorXd>& u) 
  {
    if ( (x.array() < l.array()).all() )
      return false;
    if ( (x.array() > u.array()).all() )
      return false;
    return true;
  }
  
  
  /**
   * @brief Bound x to the interval [l, u]
   * @param l lower bound
   * @param x value to be bounded
   * @param u upper bound
   * @tparam T numerical type
   * @return bounded value
   */
  template <typename T>
  inline T bound(T l, T x, T u) 
  {
    if (x < l)
      return l;
    if (x > u)
      return u;
    return x;
  }
  
  /**
   * @brief Apply upper bound such that x<=u
   * @param x value to be bounded
   * @param u upper bound
   * @tparam T numerical type
   * @return bounded value
   */
  template <typename T>
  inline T upper_bound(T x, T u) 
  {
    if (x > u)
      return u;
    return x;
  }
  
  /**
   * @brief Apply lower bound such that x>=u
   * @param l lower bound
   * @param x value to be bounded
   * @tparam T numerical type
   * @return bounded value
   */
  template <typename T>
  inline T lower_bound(T l, T x) 
  {
    if (x < l)
      return l;
    return x;
  }    
    
  /**
   * @brief Convert roll-pitch-yaw angles to a 3d rotation matrix.
   * @param rpy vector containing [roll,pitch,yaw] angles
   * @return 3x3 rotation matrix
   */  
  inline Eigen::Matrix3d rpyToRotMat(const Eigen::Ref<const RpyVector>& rpy)
  {    
    return (Eigen::AngleAxisd(rpy.coeffRef(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy.coeffRef(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy.coeffRef(0), Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  
  
  /**
   * @brief Convert a rotation matrix to roll-pitch-yaw angles
   * @param rot_mat 3x3 rotation matrix
   * @return vector contining [roll,pitch,yaw] angles
   */
  inline RpyVector rotMatToRpy(const Eigen::Ref<const Eigen::Matrix3d>& rot_mat)
  {    
    // see http://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel
    RpyVector rpy;
    rpy.coeffRef(2) = std::atan2( rot_mat.coeffRef(1,0), rot_mat.coeffRef(0,0) ); //atan2(r21,r11)
    rpy.coeffRef(1) = std::atan2( -rot_mat.coeffRef(2,0), sqrt( rot_mat.coeffRef(0,0)*rot_mat.coeffRef(0,0) + rot_mat.coeffRef(1,0)*rot_mat.coeffRef(1,0) ) ); //atan2(-r31,sqrt(r11^2 + r21^2))
    rpy.coeffRef(0) = std::atan2( rot_mat.coeffRef(2,1), rot_mat.coeffRef(2,2)  ); //atan2(r32,r33)
    
    // check for singularities
    if (fabs(rpy.coeffRef(1)-M_PI/2)<1e-4)
    {
      rpy.coeffRef(2) = 0;
      rpy.coeffRef(0) = std::atan2( rot_mat.coeffRef(0,1), rot_mat.coeffRef(1,1)  ); //atan2(r12,r22)
    } 
    else if (fabs(rpy.coeffRef(1)+M_PI/2)<1e-4)
    {
      rpy.coeffRef(2) = 0;
      rpy.coeffRef(0) = -std::atan2( rot_mat.coeffRef(0,1), rot_mat.coeffRef(1,1)  ); //-atan2(r12,r22)
    }
    return rpy;
  }
    
    
  
  /**
    * @brief Constructs an object of type T and wraps it in a std::unique_ptr. 
    * 
    * Until C++14 is not widely spread, we define our own make_unique.
    * @param args Arbitrary argument list that is compatible with the constructor list required by type \c T
    * @return std::unique_ptr of type \c T constructed with \c args
   */
  template<typename T, typename... Args>
  inline std::unique_ptr<T> make_unique(Args&&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
  
 

} // namespace phantomx


#endif /* MISC_H_ */
