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
/*----------------------------------- Include --------------------------------------*/
/************************************************************************************/
#include "mujoco_ros2_demo/ant_walk_component.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace mujoco_ros2
{

AntWalkComponent::AntWalkComponent(const rclcpp::NodeOptions & options)
: Node("ant_walker", options)
{
  const double rate = declare_parameter<double>("rate", 100.0);
  gait_freq_ = declare_parameter<double>("gait_freq", 1.5);                    // [Hz]
  hip_amp_ = declare_parameter<double>("hip_amp_deg", 25.0) * M_PI / 180.0;    // [rad]
  ankle_lift_ = declare_parameter<double>("ankle_lift_deg", 35.0) * M_PI / 180.0;
  stop_distance_ = declare_parameter<double>("stop_distance", 1.0);           // [m]
  hip_sign_ = declare_parameter<std::vector<double>>(
    "hip_sign", std::vector<double>{1.0, -1.0, 1.0, -1.0});
  leg_phase_ = declare_parameter<std::vector<double>>(
    "leg_phase", std::vector<double>{0.0, 0.0, M_PI, M_PI});

  cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(*msg);});

  start_ = std::chrono::steady_clock::now();
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {onTimer();});
  RCLCPP_INFO(get_logger(), "ant_walker started (stop_distance %.2f m)", stop_distance_);
}

void AntWalkComponent::onScan(const sensor_msgs::msg::LaserScan & scan)
{
  if (stopped_ || scan.ranges.empty()) {return;}
  const float front = scan.ranges.front();
  if (std::isfinite(front) && front < stop_distance_) {
    stopped_ = true;
    RCLCPP_INFO(get_logger(), "obstacle at %.2f m -> stop", front);
  }
}

void AntWalkComponent::onTimer()
{
  const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
  const double phase = 2.0 * M_PI * gait_freq_ * t;

  sensor_msgs::msg::JointState cmd;
  cmd.header.stamp = now();
  for (int leg = 0; leg < 4; ++leg) {
    const double p = phase + leg_phase_[leg];
    cmd.name.push_back("hip" + std::to_string(leg + 1));
    cmd.position.push_back(stopped_ ? 0.0 : hip_sign_[leg] * hip_amp_ * std::sin(p));
    cmd.name.push_back("ankle" + std::to_string(leg + 1));
    cmd.position.push_back(stopped_ ? 0.0 : -ankle_lift_ * std::max(0.0, std::cos(p)));
  }
  cmd_pub_->publish(cmd);
}

}  // namespace mujoco_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(mujoco_ros2::AntWalkComponent)
