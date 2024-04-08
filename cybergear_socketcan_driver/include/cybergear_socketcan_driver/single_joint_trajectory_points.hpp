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
  ~SingleJointTrajectoryPoints();

  void reset();
  void load(const std::string & joint_name, const trajectory_msgs::msg::JointTrajectory &);
  const Points & points();

  SingleJointTrajectoryPoints & operator=(const SingleJointTrajectoryPoints &);

private:
  Points m_trajectory_points;

  unsigned int getJointIndexFromJointNames(
    const std::string &, const std::vector<std::string> &);
};
}  // namespace cybergear_socketcan_driver
