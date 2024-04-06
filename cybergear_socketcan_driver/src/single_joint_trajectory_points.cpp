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

#include <cybergear_socketcan_driver/single_joint_trajectory_points.hpp>

#include <algorithm>
#include <iostream>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace cybergear_socketcan_driver
{
SingleJointTrajectoryPoints::SingleJointTrajectoryPoints()
: m_trajectory_points() {}

SingleJointTrajectoryPoints::SingleJointTrajectoryPoints(
  const SingleJointTrajectoryPoints & joint_trajectory)
{
  if (this != &joint_trajectory) {
    this->m_trajectory_points = joint_trajectory.m_trajectory_points;
  }
}

SingleJointTrajectoryPoints::~SingleJointTrajectoryPoints() {}

void SingleJointTrajectoryPoints::reset()
{
  m_trajectory_points.clear();
}

void SingleJointTrajectoryPoints::load(
  const std::string & joint_name,
  const trajectory_msgs::msg::JointTrajectory & joint_trajectory)
{
  const unsigned int joint_index = getJointIndexFromJointNames(
    joint_name, joint_trajectory.joint_names);

  if (joint_index >= joint_trajectory.joint_names.size()) {
    return;
  }
  m_trajectory_points.resize(joint_trajectory.points.size());
  int point_index = 0;

  for (const auto & point : joint_trajectory.points) {
    if (point.positions.size() > joint_index) {
      m_trajectory_points[point_index].position = point.positions[joint_index];
    }
    if (point.velocities.size() > joint_index) {
      m_trajectory_points[point_index].velocity = point.velocities[joint_index];
    }
    if (point.accelerations.size() > joint_index) {
      m_trajectory_points[point_index].acceleration = point.accelerations[joint_index];
    }
    if (point.effort.size() > joint_index) {
      m_trajectory_points[point_index].effort = point.effort[joint_index];
    }
    m_trajectory_points[point_index].time_from_start = point.time_from_start;

    point_index++;
  }
}

const SingleJointTrajectoryPoints::Points & SingleJointTrajectoryPoints::points()
{
  return m_trajectory_points;
}

SingleJointTrajectoryPoints & SingleJointTrajectoryPoints::operator=(
  const SingleJointTrajectoryPoints & joint_trajectory)
{
  if (this != &joint_trajectory) {
    this->m_trajectory_points = joint_trajectory.m_trajectory_points;
  }
  return *this;
}

unsigned int SingleJointTrajectoryPoints::getJointIndexFromJointNames(
  const std::string & joint_name,
  const std::vector<std::string> & joint_names)
{
  unsigned int joint_index = 0;

  for (const auto & name : joint_names) {
    if (joint_name == name) {
      return joint_index;
    }
    joint_index++;
  }
  return joint_names.size();
}
}  // namespace cybergear_socketcan_driver
