/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Common conversion functions between geometry messages, Eigen types, and yaw.

#ifndef MAV_MSGS_COMMON_H
#define MAV_MSGS_COMMON_H

#include <Eigen/Geometry>

namespace mav_msgs {

const double kNumNanosecondsPerSecond = 1.e9;

/// Magnitude of Earth's gravitational field at specific height [m] and latitude
/// [rad] (from wikipedia).
inline double MagnitudeOfGravity(const double height,
                                 const double latitude_radians) {
  // gravity calculation constants
  const double kGravity_0 = 9.780327;
  const double kGravity_a = 0.0053024;
  const double kGravity_b = 0.0000058;
  const double kGravity_c = 3.155 * 1e-7;

  double sin_squared_latitude = sin(latitude_radians) * sin(latitude_radians);
  double sin_squared_twice_latitude =
      sin(2 * latitude_radians) * sin(2 * latitude_radians);
  return kGravity_0 * ((1 + kGravity_a * sin_squared_latitude -
                        kGravity_b * sin_squared_twice_latitude) -
                       kGravity_c * height);
}

/**
 * \brief Extracts the yaw part from a quaternion, using RPY / euler (z-y'-z'')
 * angles.
 * RPY rotates about the fixed axes in the order x-y-z,
 * which is the same as euler angles in the order z-y'-x''.
 */
inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
  {
    assert(euler_angles != NULL);

    *euler_angles << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
        atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
}

// Calculate the nominal rotor rates given the MAV mass, allocation matrix,
// angular velocity, angular acceleration, and body acceleration (normalized
// thrust).
//
// [torques, thrust]' = A * n^2, where
// torques = J * ang_acc + ang_vel x J
// thrust = m * norm(acc)
//
// The allocation matrix A has of a hexacopter is:
// A = K * B, where
// K = diag(l*c_T, l*c_T, c_M, c_T),
//     [ s  1  s -s -1 -s]
// B = [-c  0  c  c  0 -c]
//     [-1  1 -1  1 -1  1]
//     [ 1  1  1  1  1  1],
// l: arm length
// c_T: thrust constant
// c_M: moment constant
// s: sin(30°)
// c: cos(30°)
//
// The inverse can be computed computationally efficient:
// A^-1 \approx B^pseudo * K^-1
inline void getSquaredRotorSpeedsFromAllocationAndState(
    const Eigen::MatrixXd& allocation_inv, const Eigen::Vector3d& inertia,
    double mass, const Eigen::Vector3d& angular_velocity_B,
    const Eigen::Vector3d& angular_acceleration_B,
    const Eigen::Vector3d& acceleration_B,
    Eigen::VectorXd* rotor_rates_squared) {
  const Eigen::Vector3d torque =
      inertia.asDiagonal() * angular_acceleration_B +
      angular_velocity_B.cross(inertia.asDiagonal() * angular_velocity_B);
  const double thrust_force = mass * acceleration_B.norm();
  Eigen::Vector4d input;
  input << torque, thrust_force;
  *rotor_rates_squared = allocation_inv * input;
}

inline double nanosecondsToSeconds(int64_t nanoseconds) {
  double seconds = nanoseconds / kNumNanosecondsPerSecond;
  return seconds;
}

inline int64_t secondsToNanoseconds(double seconds) {
  int64_t nanoseconds =
      static_cast<int64_t>(seconds * kNumNanosecondsPerSecond);
  return nanoseconds;
}

}  // namespace mav_msgs

#endif  // MAV_MSGS_COMMON_H
