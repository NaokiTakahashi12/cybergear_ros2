// MIT License
//
// Copyright (c) 2024 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "single_joint_trajectory_point.hpp"

namespace cybergear_socketcan_driver
{
class SingleJointTrajectoryPoints
{
public:
  using UniquePtr = std::unique_ptr<SingleJointTrajectoryPoints>;
  using SharedPtr = std::shared_ptr<SingleJointTrajectoryPoints>;

  using Points = std::vector<SingleJointTrajectoryPoint>;

  SingleJointTrajectoryPoints();
  SingleJointTrajectoryPoints(const SingleJointTrajectoryPoints &);
  SingleJointTrajectoryPoints(SingleJointTrajectoryPoints &&) = delete;
  ~SingleJointTrajectoryPoints();

  void reset();

  void initTrajectoryPoint(const sensor_msgs::msg::JointState &);
  void load(const std::string & joint_name, const trajectory_msgs::msg::JointTrajectory &);

  float getLerpPosition(const builtin_interfaces::msg::Time &) const;
  float getLerpVelocity(const builtin_interfaces::msg::Time &) const;
  float getLerpEffort(const builtin_interfaces::msg::Time &) const;

  const Points & points() const;

  SingleJointTrajectoryPoints & operator=(const SingleJointTrajectoryPoints &);
  SingleJointTrajectoryPoints & operator=(SingleJointTrajectoryPoints &&) = delete;

private:
  SingleJointTrajectoryPoint start_trajectory_point_;
  Points trajectory_points_;

  rclcpp::Time start_trajectory_time_;
  std::vector<float> trajectory_durations_from_recived_;

  static unsigned int getJointIndexFromJointNames(
    const std::string &, const std::vector<std::string> &);
};
}  // namespace cybergear_socketcan_driver
