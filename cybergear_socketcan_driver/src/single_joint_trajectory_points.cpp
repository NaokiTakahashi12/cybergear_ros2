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

#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace cybergear_socketcan_driver
{
SingleJointTrajectoryPoints::SingleJointTrajectoryPoints()
: m_start_trajectory_point(),
  m_trajectory_points(),
  m_start_trajectory_time(),
  m_trajectory_durations_from_recived()
{
}

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

void SingleJointTrajectoryPoints::initTrajectoryPoint(
  const sensor_msgs::msg::JointState & joint_state)
{
  m_start_trajectory_point.position = joint_state.position[0];
  m_start_trajectory_point.velocity = joint_state.velocity[0];
  m_start_trajectory_point.effort = joint_state.effort[0];
}

void SingleJointTrajectoryPoints::load(
  const std::string & joint_name,
  const trajectory_msgs::msg::JointTrajectory & joint_trajectory)
{
  const unsigned int joint_index = getJointIndexFromJointNames(
    joint_name, joint_trajectory.joint_names);
  const unsigned int joint_trajectory_points_size = joint_trajectory.points.size();

  if (joint_index >= joint_trajectory.joint_names.size()) {
    return;
  }
  m_trajectory_durations_from_recived.resize(joint_trajectory_points_size);
  m_trajectory_points.resize(joint_trajectory_points_size);
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

  for (unsigned int i = 0; i < joint_trajectory_points_size; ++i) {
    m_trajectory_durations_from_recived[i] = 0.0;
    for (unsigned int j = 0; j <= i; ++j) {
      rclcpp::Duration duration(joint_trajectory.points[j].time_from_start);
      m_trajectory_durations_from_recived[i] += duration.seconds();
    }
  }
  m_start_trajectory_time = rclcpp::Time(joint_trajectory.header.stamp);
}

float SingleJointTrajectoryPoints::getLerpPosition(const builtin_interfaces::msg::Time & time)
{
  if (m_trajectory_durations_from_recived.empty()) {
    return 0.0f;
  }
  rclcpp::Time control_time(time);
  const double time_from_recived = (control_time - m_start_trajectory_time).seconds();

  if (m_trajectory_durations_from_recived.back() < time_from_recived) {
    return m_trajectory_points.back().position;
  }
  unsigned int point_index = 0;

  for (const auto & duration : m_trajectory_durations_from_recived) {
    if (duration > time_from_recived) {
      break;
    }
    point_index++;
  }
  float start_point;

  if (point_index == 0) {
    start_point = m_start_trajectory_point.position;
  } else {
    start_point = m_trajectory_points[point_index - 1].position;
  }
  const float dest_delta = m_trajectory_points[point_index].position - start_point;
  const double point_duration = rclcpp::Duration(
    m_trajectory_points[point_index].time_from_start).seconds();
  float normalized_duration;

  if (point_duration == 0.0) {
    normalized_duration = 1.0;
  } else {
    const double time_left = m_trajectory_durations_from_recived[point_index] - time_from_recived;
    normalized_duration = 1 - (time_left) / point_duration;
  }
  return normalized_duration * dest_delta + start_point;
}

float SingleJointTrajectoryPoints::getLerpVelocity(const builtin_interfaces::msg::Time & time)
{
  if (m_trajectory_durations_from_recived.empty()) {
    return 0.0f;
  }
  rclcpp::Time control_time(time);
  const double time_from_recived = (control_time - m_start_trajectory_time).seconds();

  if (m_trajectory_durations_from_recived.back() < time_from_recived) {
    return m_trajectory_points.back().velocity;
  }
  unsigned int point_index = 0;

  for (const auto & duration : m_trajectory_durations_from_recived) {
    if (duration > time_from_recived) {
      break;
    }
    point_index++;
  }
  float start_point;

  if (point_index == 0) {
    start_point = m_start_trajectory_point.velocity;
  } else {
    start_point = m_trajectory_points[point_index - 1].velocity;
  }
  const float dest_delta = m_trajectory_points[point_index].velocity - start_point;
  const double point_duration = rclcpp::Duration(
    m_trajectory_points[point_index].time_from_start).seconds();
  float normalized_duration;

  if (point_duration == 0.0) {
    normalized_duration = 1.0;
  } else {
    const double time_left = m_trajectory_durations_from_recived[point_index] - time_from_recived;
    normalized_duration = 1 - (time_left) / point_duration;
  }
  return normalized_duration * dest_delta + start_point;
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
