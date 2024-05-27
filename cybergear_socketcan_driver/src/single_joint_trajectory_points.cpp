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

#include <algorithm>
#include <cmath>

#include <builtin_interfaces/msg/duration.hpp>
#include <cybergear_socketcan_driver/single_joint_trajectory_points.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace cybergear_socketcan_driver
{
SingleJointTrajectoryPoints::SingleJointTrajectoryPoints() {}

SingleJointTrajectoryPoints::SingleJointTrajectoryPoints(
  const SingleJointTrajectoryPoints & joint_trajectory)
{
  if (this != &joint_trajectory) {
    this->trajectory_points_ = joint_trajectory.trajectory_points_;
  }
}

SingleJointTrajectoryPoints::~SingleJointTrajectoryPoints() {}

void SingleJointTrajectoryPoints::reset()
{
  trajectory_points_.clear();
}

void SingleJointTrajectoryPoints::initTrajectoryPoint(
  const sensor_msgs::msg::JointState & joint_state)
{
  start_trajectory_point_.position = static_cast<float>(joint_state.position[0]);
  start_trajectory_point_.velocity = static_cast<float>(joint_state.velocity[0]);
  start_trajectory_point_.effort = static_cast<float>(joint_state.effort[0]);
}

void SingleJointTrajectoryPoints::load(
  const std::string & joint_name, const trajectory_msgs::msg::JointTrajectory & joint_trajectory)
{
  const unsigned int joint_index =
    getJointIndexFromJointNames(joint_name, joint_trajectory.joint_names);
  const unsigned int joint_trajectory_points_size = joint_trajectory.points.size();

  if (joint_index >= joint_trajectory.joint_names.size()) {
    return;
  }
  trajectory_durations_from_recived_.resize(joint_trajectory_points_size);
  trajectory_points_.resize(joint_trajectory_points_size);
  int point_index = 0;

  for (const auto & point : joint_trajectory.points) {
    if (point.positions.size() > joint_index) {
      trajectory_points_[point_index].position = static_cast<float>(point.positions[joint_index]);
    }
    if (point.velocities.size() > joint_index) {
      trajectory_points_[point_index].velocity = static_cast<float>(point.velocities[joint_index]);
    }
    if (point.accelerations.size() > joint_index) {
      trajectory_points_[point_index].acceleration =
        static_cast<float>(point.accelerations[joint_index]);
    }
    if (point.effort.size() > joint_index) {
      trajectory_points_[point_index].effort = static_cast<float>(point.effort[joint_index]);
    }
    trajectory_points_[point_index].time_from_start = point.time_from_start;
    point_index++;
  }

  for (unsigned int i = 0; i < joint_trajectory_points_size; ++i) {
    trajectory_durations_from_recived_[i] = 0.0;
    for (unsigned int j = 0; j <= i; ++j) {
      rclcpp::Duration duration(joint_trajectory.points[j].time_from_start);
      trajectory_durations_from_recived_[i] += static_cast<float>(duration.seconds());
    }
  }
  start_trajectory_time_ = rclcpp::Time(joint_trajectory.header.stamp);
}

float SingleJointTrajectoryPoints::getLerpPosition(const builtin_interfaces::msg::Time & time) const
{
  if (trajectory_durations_from_recived_.empty()) {
    return 0.0F;
  }
  rclcpp::Time control_time(time);
  const float time_from_recived =
    static_cast<float>((control_time - start_trajectory_time_).seconds());

  if (trajectory_durations_from_recived_.back() < time_from_recived) {
    return trajectory_points_.back().position;
  }
  unsigned int point_index = 0;

  for (const auto & duration : trajectory_durations_from_recived_) {
    if (duration > time_from_recived) {
      break;
    }
    point_index++;
  }
  float start_point = NAN;

  if (point_index == 0) {
    start_point = start_trajectory_point_.position;
  } else {
    start_point = trajectory_points_[point_index - 1].position;
  }
  const float dest_delta = trajectory_points_[point_index].position - start_point;
  const float point_duration =
    static_cast<float>(rclcpp::Duration(trajectory_points_[point_index].time_from_start).seconds());
  float normalized_duration = NAN;

  if (point_duration == 0.0) {
    normalized_duration = 1.0F;
  } else {
    const float time_left = trajectory_durations_from_recived_[point_index] - time_from_recived;
    normalized_duration = 1.0F - (time_left) / point_duration;
  }
  return normalized_duration * dest_delta + start_point;
}

float SingleJointTrajectoryPoints::getLerpVelocity(const builtin_interfaces::msg::Time & time) const
{
  if (trajectory_durations_from_recived_.empty()) {
    return 0.0F;
  }
  rclcpp::Time control_time(time);
  const float time_from_recived =
    static_cast<float>((control_time - start_trajectory_time_).seconds());

  if (trajectory_durations_from_recived_.back() < time_from_recived) {
    return trajectory_points_.back().velocity;
  }
  unsigned int point_index = 0;

  for (const auto & duration : trajectory_durations_from_recived_) {
    if (duration > time_from_recived) {
      break;
    }
    point_index++;
  }
  float start_point = NAN;

  if (point_index == 0) {
    start_point = start_trajectory_point_.velocity;
  } else {
    start_point = trajectory_points_[point_index - 1].velocity;
  }
  const float dest_delta = trajectory_points_[point_index].velocity - start_point;
  const float point_duration =
    static_cast<float>(rclcpp::Duration(trajectory_points_[point_index].time_from_start).seconds());
  float normalized_duration = NAN;

  if (point_duration == 0.0) {
    normalized_duration = 1.0F;
  } else {
    const float time_left = trajectory_durations_from_recived_[point_index] - time_from_recived;
    normalized_duration = 1.0F - (time_left) / point_duration;
  }
  return normalized_duration * dest_delta + start_point;
}

float SingleJointTrajectoryPoints::getLerpEffort(const builtin_interfaces::msg::Time & time) const
{
  if (trajectory_durations_from_recived_.empty()) {
    return 0.0F;
  }
  rclcpp::Time control_time(time);
  const float time_from_recived =
    static_cast<float>((control_time - start_trajectory_time_).seconds());

  if (trajectory_durations_from_recived_.back() < time_from_recived) {
    return trajectory_points_.back().effort;
  }
  unsigned int point_index = 0;

  for (const auto & duration : trajectory_durations_from_recived_) {
    if (duration > time_from_recived) {
      break;
    }
    point_index++;
  }
  float start_point = NAN;

  if (point_index == 0) {
    start_point = start_trajectory_point_.effort;
  } else {
    start_point = trajectory_points_[point_index - 1].effort;
  }
  const float dest_delta = trajectory_points_[point_index].effort - start_point;
  const float point_duration =
    static_cast<float>(rclcpp::Duration(trajectory_points_[point_index].time_from_start).seconds());
  float normalized_duration = NAN;

  if (point_duration == 0.0) {
    normalized_duration = 1.0F;
  } else {
    const float time_left = trajectory_durations_from_recived_[point_index] - time_from_recived;
    normalized_duration = 1.0F - (time_left) / point_duration;
  }
  return normalized_duration * dest_delta + start_point;
}

const SingleJointTrajectoryPoints::Points & SingleJointTrajectoryPoints::points() const
{
  return trajectory_points_;
}

SingleJointTrajectoryPoints & SingleJointTrajectoryPoints::operator=(
  const SingleJointTrajectoryPoints & joint_trajectory)
{
  if (this != &joint_trajectory) {
    this->trajectory_points_ = joint_trajectory.trajectory_points_;
  }
  return *this;
}

unsigned int SingleJointTrajectoryPoints::getJointIndexFromJointNames(
  const std::string & joint_name, const std::vector<std::string> & joint_names)
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
