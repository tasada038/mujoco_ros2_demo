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
#ifndef MUJOCO_ROS2_DEMO__MUJOCO_VIEW_COMPONENT_HPP_
#define MUJOCO_ROS2_DEMO__MUJOCO_VIEW_COMPONENT_HPP_

/************************************************************************************/
/*----------------------------------- Include --------------------------------------*/
/************************************************************************************/
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace mujoco_ros2
{

class MujocoViewComponent : public rclcpp::Node
{
public:
  explicit MujocoViewComponent(const rclcpp::NodeOptions & options);
  ~MujocoViewComponent() override;

private:
  struct Camera
  {
    int id;
    std::string name;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb, depth;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
  };

  // names
  std::string name(mjtObj type, int id) const;
  std::string bodyFrame(int body) const;

  // ROS setup
  void setupRos();
  std::string buildUrdf() const;
  void onCommand(const sensor_msgs::msg::JointState & msg);

  // simulation + rendering (sim thread)
  void simLoop();
  bool initGl();
  void onMouseMove(double x, double y);
  void render();

  // publishers (sim thread)
  geometry_msgs::msg::TransformStamped makeTf(
    const std::string & parent, const std::string & child, const mjtNum * pos, const mjtNum * quat) const;
  void publishState();
  void publishCameras();

  // parameters
  std::string model_path_, lidar_prefix_, imu_prefix_, world_frame_;
  double publish_rate_, camera_rate_, lidar_range_max_;
  int camera_width_, camera_height_;
  std::vector<std::string> camera_names_;
  bool camera_inset_;

  // MuJoCo
  mjModel * mj_model_ = nullptr;   // model loaded from the MJCF (XML) file
  mjData * mj_data_ = nullptr;     // simulation state
  mjvCamera free_cam_;             // mouse-controlled window camera
  mjvOption window_opt_, camera_opt_;
  mjvScene scene_;
  mjrContext render_ctx_;
  GLFWwindow * window_ = nullptr;
  double last_x_ = 0, last_y_ = 0;
  int inset_index_ = 0;            // camera shown at the bottom right (-1 = none)
  std::vector<unsigned char> rgb_buf_;
  std::vector<float> depth_buf_;

  // threading / commands
  std::thread sim_thread_;
  std::atomic<bool> running_{false}, paused_{false}, reset_{false};
  std::mutex cmd_mutex_;
  std::map<int, double> cmd_;                 // actuator id -> ctrl
  std::map<std::string, int> joint_to_act_;   // joint or actuator name -> actuator id

  // sensors
  std::vector<int> joints_, lidar_;
  int imu_acc_ = -1, imu_gyro_ = -1, imu_quat_ = -1;
  rclcpp::Time stamp_;

  // ROS interfaces
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::vector<std::pair<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>> touch_;
  std::vector<std::pair<int, rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr>> contact_;
  std::vector<Camera> cams_;
};

}  // namespace mujoco_ros2

#endif  // MUJOCO_ROS2_DEMO__MUJOCO_VIEW_COMPONENT_HPP_
