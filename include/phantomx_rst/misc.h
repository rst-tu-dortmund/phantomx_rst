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
