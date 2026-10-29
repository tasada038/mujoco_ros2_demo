//-----------------------------------------------------------------------------------
// MIT License

// Copyright (c) 2026 Takumi Asada

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//-----------------------------------------------------------------------------------

/************************************************************************************/
/*------------------------------------ Define --------------------------------------*/
/************************************************************************************/
#ifndef MUJOCO_ROS2_DEMO__ANT_WALK_COMPONENT_HPP_
#define MUJOCO_ROS2_DEMO__ANT_WALK_COMPONENT_HPP_

/************************************************************************************/
/*----------------------------------- Include --------------------------------------*/
/************************************************************************************/
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace mujoco_ros2
{

class AntWalkComponent : public rclcpp::Node
{
public:
  explicit AntWalkComponent(const rclcpp::NodeOptions & options);

private:
  void onScan(const sensor_msgs::msg::LaserScan & scan);
  void onTimer();

  double gait_freq_, hip_amp_, ankle_lift_, stop_distance_;
  std::vector<double> hip_sign_, leg_phase_;
  bool stopped_ = false;
  std::chrono::steady_clock::time_point start_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mujoco_ros2

#endif  // MUJOCO_ROS2_DEMO__ANT_WALK_COMPONENT_HPP_
