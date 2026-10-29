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
#include "mujoco_ros2_demo/mujoco_view_component.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace mujoco_ros2
{
namespace
{
// mjv_moveCamera dropped the mjvScene argument in newer MuJoCo versions: support both.
template<class F>
void moveCamera(F f, const mjModel * m, int action, mjtNum dx, mjtNum dy, mjvScene * scn, mjvCamera * cam)
{
  if constexpr (std::is_invocable_v<F, const mjModel *, int, mjtNum, mjtNum, mjvCamera *>) {
    f(m, action, dx, dy, cam);
  } else {
    f(m, action, dx, dy, scn, cam);
  }
}

// MuJoCo quaternion (w, x, y, z) -> URDF "r p y"
std::string quatToRpy(const mjtNum * q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  const double roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const double pitch = std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0));
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  std::ostringstream s;
  s << roll << " " << pitch << " " << yaw;
  return s.str();
}
}  // namespace


/************************************************************************************/
/*--------------------------------- Constructor ------------------------------------*/
/************************************************************************************/
MujocoViewComponent::MujocoViewComponent(const rclcpp::NodeOptions & options)
: Node("mujoco_view", options)
{
  // model_path: absolute path, or a file name in <this package>/models
  model_path_ = declare_parameter<std::string>("model_path", "ant_sensors.xml");
  if (!model_path_.empty() && model_path_.front() != '/') {
    model_path_ = ament_index_cpp::get_package_share_directory("mujoco_ros2_demo") +
      "/models/" + model_path_;
  }
  publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
  camera_names_ = declare_parameter<std::vector<std::string>>(
    "camera_names", std::vector<std::string>{});          // empty = all cameras in the model
  camera_rate_ = declare_parameter<double>("camera_rate", 10.0);
  camera_width_ = declare_parameter<int>("camera_width", 640);
  camera_height_ = declare_parameter<int>("camera_height", 480);
  camera_inset_ = declare_parameter<bool>("camera_inset", true);  // camera image in the window
  lidar_prefix_ = declare_parameter<std::string>("lidar_prefix", "lidar");
  lidar_range_max_ = declare_parameter<double>("lidar_range_max", 10.0);
  imu_prefix_ = declare_parameter<std::string>("imu_prefix", "imu");
  world_frame_ = declare_parameter<std::string>("world_frame", "world");

  char err[1000] = "";
  mj_model_ = mj_loadXML(model_path_.c_str(), nullptr, err, sizeof(err));
  if (!mj_model_) {
    RCLCPP_FATAL(get_logger(), "Failed to load '%s': %s", model_path_.c_str(), err);
    throw std::runtime_error("model load failed");
  }
  mj_data_ = mj_makeData(mj_model_);
  mj_forward(mj_model_, mj_data_);

  setupRos();
  running_ = true;
  sim_thread_ = std::thread(&MujocoViewComponent::simLoop, this);
  RCLCPP_INFO(get_logger(), "Loaded %s (nq=%d, nu=%d, nsensor=%d, cameras=%zu)",
    model_path_.c_str(), static_cast<int>(mj_model_->nq), static_cast<int>(mj_model_->nu),
    static_cast<int>(mj_model_->nsensor), cams_.size());
}

/************************************************************************************/
/*--------------------------------- Destructor -------------------------------------*/
/************************************************************************************/
MujocoViewComponent::~MujocoViewComponent()
{
  running_ = false;
  if (sim_thread_.joinable()) {sim_thread_.join();}
  mj_deleteData(mj_data_);
  mj_deleteModel(mj_model_);
}

std::string MujocoViewComponent::name(mjtObj type, int id) const
{
  const char * n = mj_id2name(mj_model_, type, id);
  return n ? n : "";
}

std::string MujocoViewComponent::bodyFrame(int body) const
{
  if (body == 0) {return world_frame_;}
  const std::string n = name(mjOBJ_BODY, body);
  return n.empty() ? "body_" + std::to_string(body) : n;
}

/************************************************************************************/
/*------------------------------ Function: setup ROS ------------------------------*/
/************************************************************************************/
void MujocoViewComponent::setupRos()
{
  const mjModel * m = mj_model_;
  tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_commands", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onCommand(*msg);});

  // URDF for RViz RobotModel (latched so that RViz started later still receives it).
  // Intra-process is disabled here because it does not support transient local durability.
  rclcpp::PublisherOptions description_opts;
  description_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  description_pub_ = create_publisher<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(), description_opts);
  std_msgs::msg::String urdf;
  urdf.data = buildUrdf();
  description_pub_->publish(urdf);

  // Actuators driving a joint, looked up by joint name and by actuator name
  for (int a = 0; a < m->nu; ++a) {
    if (m->actuator_trntype[a] == mjTRN_JOINT) {
      joint_to_act_[name(mjOBJ_JOINT, m->actuator_trnid[2 * a])] = a;
    }
    joint_to_act_[name(mjOBJ_ACTUATOR, a)] = a;
  }
  for (int j = 0; j < m->njnt; ++j) {
    if (m->jnt_type[j] == mjJNT_HINGE || m->jnt_type[j] == mjJNT_SLIDE) {joints_.push_back(j);}
  }

  // Sensors grouped by type / name
  for (int s = 0; s < m->nsensor; ++s) {
    const std::string n = name(mjOBJ_SENSOR, s);
    const int type = m->sensor_type[s];
    if (type == mjSENS_RANGEFINDER && n.rfind(lidar_prefix_, 0) == 0) {
      lidar_.push_back(s);
    } else if (type == mjSENS_TOUCH) {
      touch_.emplace_back(s, create_publisher<std_msgs::msg::Float32>(n, 10));
    } else if (type == mjSENS_CONTACT && m->sensor_dim[s] == 4 && m->sensor_objtype[s] == mjOBJ_GEOM) {
      contact_.emplace_back(s, create_publisher<geometry_msgs::msg::WrenchStamped>(n, 10));
    }
  }
  imu_acc_ = mj_name2id(m, mjOBJ_SENSOR, (imu_prefix_ + "_acc").c_str());
  imu_gyro_ = mj_name2id(m, mjOBJ_SENSOR, (imu_prefix_ + "_gyro").c_str());
  imu_quat_ = mj_name2id(m, mjOBJ_SENSOR, (imu_prefix_ + "_quat").c_str());
  if (imu_acc_ >= 0 || imu_gyro_ >= 0 || imu_quat_ >= 0) {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  }
  if (!lidar_.empty()) {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  }

  // Cameras: the listed ones, or every camera in the model
  if (camera_names_.empty()) {
    for (int c = 0; c < m->ncam; ++c) {camera_names_.push_back(name(mjOBJ_CAMERA, c));}
  }
  for (const auto & cam : camera_names_) {
    const int id = mj_name2id(m, mjOBJ_CAMERA, cam.c_str());
    if (cam.empty() || id < 0) {
      RCLCPP_WARN(get_logger(), "camera '%s' not found (cameras need a name)", cam.c_str());
      continue;
    }
    cams_.push_back({id, cam,
        create_publisher<sensor_msgs::msg::Image>(cam + "/image_raw", 1),
        create_publisher<sensor_msgs::msg::Image>(cam + "/depth", 1),
        create_publisher<sensor_msgs::msg::CameraInfo>(cam + "/camera_info", 1)});
  }
  inset_index_ = (camera_inset_ && !cams_.empty()) ? 0 : -1;
}

/************************************************************************************/
/*---------------------------------- Build URDF  -----------------------------------*/
/************************************************************************************/
std::string MujocoViewComponent::buildUrdf() const
{
  const mjModel * m = mj_model_;
  std::ostringstream u;
  u << "<?xml version=\"1.0\"?>\n<robot name=\"" << (m->names + 0) << "\">\n";
  for (int b = 0; b < m->nbody; ++b) {
    u << "  <link name=\"" << bodyFrame(b) << "\">\n";
    for (int g = m->body_geomadr[b]; g >= 0 && g < m->body_geomadr[b] + m->body_geomnum[b]; ++g) {
      const mjtNum * s = m->geom_size + 3 * g;
      const float * rgba = m->geom_matid[g] >= 0 ? m->mat_rgba + 4 * m->geom_matid[g] : m->geom_rgba + 4 * g;
      if (rgba[3] <= 0.0f) {continue;}
      std::ostringstream shape;
      std::vector<double> sphere_z;  // extra spheres (capsule ends) along the local z axis
      switch (m->geom_type[g]) {
        case mjGEOM_BOX:
          shape << "<box size=\"" << 2 * s[0] << " " << 2 * s[1] << " " << 2 * s[2] << "\"/>";
          break;
        case mjGEOM_SPHERE:
          shape << "<sphere radius=\"" << s[0] << "\"/>";
          break;
        case mjGEOM_ELLIPSOID:
          shape << "<sphere radius=\"" << std::max({s[0], s[1], s[2]}) << "\"/>";
          break;
        case mjGEOM_CYLINDER:
          shape << "<cylinder radius=\"" << s[0] << "\" length=\"" << 2 * s[1] << "\"/>";
          break;
        case mjGEOM_CAPSULE:
          shape << "<cylinder radius=\"" << s[0] << "\" length=\"" << 2 * s[1] << "\"/>";
          sphere_z = {s[1], -s[1]};
          break;
        default:
          continue;  // plane, mesh, hfield, ...
      }
      const mjtNum * p = m->geom_pos + 3 * g;
      const mjtNum * q = m->geom_quat + 4 * g;
      const std::string color = std::to_string(rgba[0]) + " " + std::to_string(rgba[1]) + " " +
        std::to_string(rgba[2]) + " " + std::to_string(rgba[3]);
      auto visual = [&](const mjtNum * pos, const std::string & geometry, int k) {
          u << "    <visual><origin xyz=\"" << pos[0] << " " << pos[1] << " " << pos[2]
            << "\" rpy=\"" << quatToRpy(q) << "\"/><geometry>" << geometry << "</geometry>"
            << "<material name=\"g" << g << "_" << k << "\"><color rgba=\"" << color << "\"/></material></visual>\n";
        };
      visual(p, shape.str(), 0);
      for (size_t k = 0; k < sphere_z.size(); ++k) {  // capsule end caps
        mjtNum axis[3], offset[3] = {0, 0, sphere_z[k]}, pos[3];
        mju_rotVecQuat(axis, offset, q);
        mju_add3(pos, p, axis);
        visual(pos, "<sphere radius=\"" + std::to_string(s[0]) + "\"/>", static_cast<int>(k) + 1);
      }
    }
    u << "  </link>\n";
    if (b > 0) {
      const mjtNum * p = m->body_pos + 3 * b;
      u << "  <joint name=\"" << bodyFrame(b) << "_joint\" type=\"fixed\"><parent link=\""
        << bodyFrame(m->body_parentid[b]) << "\"/><child link=\"" << bodyFrame(b)
        << "\"/><origin xyz=\"" << p[0] << " " << p[1] << " " << p[2] << "\" rpy=\""
        << quatToRpy(m->body_quat + 4 * b) << "\"/></joint>\n";
    }
  }
  u << "</robot>\n";
  return u.str();
}

void MujocoViewComponent::onCommand(const sensor_msgs::msg::JointState & msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  const bool use_pos = !msg.position.empty();
  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = joint_to_act_.find(msg.name[i]);
    if (it == joint_to_act_.end()) {continue;}
    if (use_pos && i < msg.position.size()) {
      cmd_[it->second] = msg.position[i];
    } else if (!use_pos && i < msg.effort.size()) {
      cmd_[it->second] = msg.effort[i];
    }
  }
}

/************************************************************************************/
/*---------------------------- Function: Simulatin Loop ----------------------------*/
/************************************************************************************/
void MujocoViewComponent::simLoop()
{
  if (!initGl()) {return;}
  const double pub_dt = 1.0 / publish_rate_;
  const double cam_dt = 1.0 / camera_rate_;
  double next_pub = 0.0, next_cam = 0.0;
  auto last = std::chrono::steady_clock::now();

  while (running_ && rclcpp::ok() && !glfwWindowShouldClose(window_)) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::min(std::chrono::duration<double>(now - last).count(), 0.05);
    last = now;

    if (reset_) {
      mj_resetData(mj_model_, mj_data_);
      mj_forward(mj_model_, mj_data_);
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_.clear();
      }
      next_pub = next_cam = 0.0;
      reset_ = false;
    }
    if (!paused_) {
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (const auto & [a, v] : cmd_) {mj_data_->ctrl[a] = v;}
      }
      const double target = mj_data_->time + elapsed;  // advance simulation in real time
      while (mj_data_->time < target) {
        mj_step(mj_model_, mj_data_);
        if (mj_data_->time >= next_pub) {  // publish at publish_rate (simulation time)
          publishState();
          next_pub = mj_data_->time + pub_dt;
        }
      }
    } else {
      publishState();
    }
    if (!cams_.empty() && mj_data_->time >= next_cam) {
      publishCameras();
      next_cam = mj_data_->time + cam_dt;
    }
    render();
    glfwPollEvents();
  }
  mjv_freeScene(&scene_);
  mjr_freeContext(&render_ctx_);
  glfwDestroyWindow(window_);
  glfwTerminate();
}

/************************************************************************************/
/*----------------------------- Function: Init OpenGL ------------------------------*/
/************************************************************************************/
bool MujocoViewComponent::initGl()
{
  if (!glfwInit()) {
    RCLCPP_ERROR(get_logger(), "glfwInit failed");
    return false;
  }
  window_ = glfwCreateWindow(1280, 800, ("MuJoCo: " + model_path_).c_str(), nullptr, nullptr);
  if (!window_) {
    RCLCPP_ERROR(get_logger(), "glfwCreateWindow failed");
    return false;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  glfwSetWindowUserPointer(window_, this);
  glfwSetCursorPosCallback(window_, [](GLFWwindow * w, double x, double y) {
      static_cast<MujocoViewComponent *>(glfwGetWindowUserPointer(w))->onMouseMove(x, y);
    });
  glfwSetMouseButtonCallback(window_, [](GLFWwindow * w, int, int, int) {
      auto * self = static_cast<MujocoViewComponent *>(glfwGetWindowUserPointer(w));
      glfwGetCursorPos(w, &self->last_x_, &self->last_y_);
    });
  glfwSetScrollCallback(window_, [](GLFWwindow * w, double, double dy) {
      auto * self = static_cast<MujocoViewComponent *>(glfwGetWindowUserPointer(w));
      moveCamera(mjv_moveCamera, self->mj_model_, mjMOUSE_ZOOM, 0, -0.05 * dy,
        &self->scene_, &self->free_cam_);
    });
  glfwSetKeyCallback(window_, [](GLFWwindow * w, int key, int, int act, int) {
      auto * self = static_cast<MujocoViewComponent *>(glfwGetWindowUserPointer(w));
      if (act != GLFW_PRESS) {return;}
      if (key == GLFW_KEY_SPACE) {self->paused_ = !self->paused_;}
      if (key == GLFW_KEY_BACKSPACE) {self->reset_ = true;}
      if (key == GLFW_KEY_C && !self->cams_.empty()) {  // next camera ... -> none -> first
        const int n = static_cast<int>(self->cams_.size());
        self->inset_index_ = self->inset_index_ + 1 >= n ? -1 : self->inset_index_ + 1;
      }
    });

  mjv_defaultFreeCamera(mj_model_, &free_cam_);  // initial view from <statistic> / <visual><global>
  mjv_defaultOption(&window_opt_);
  mjv_defaultOption(&camera_opt_);
  camera_opt_.flags[mjVIS_RANGEFINDER] = 0;  // keep LiDAR rays out of camera images
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&render_ctx_);
  mjv_makeScene(mj_model_, &scene_, 2000);
  mjr_makeContext(mj_model_, &render_ctx_, mjFONTSCALE_150);

  // Camera images are rendered into the offscreen buffer (<visual><global offwidth/offheight>)
  camera_width_ = std::min(camera_width_, static_cast<int>(render_ctx_.offWidth));
  camera_height_ = std::min(camera_height_, static_cast<int>(render_ctx_.offHeight));
  rgb_buf_.resize(3 * camera_width_ * camera_height_);
  depth_buf_.resize(camera_width_ * camera_height_);
  return true;
}

void MujocoViewComponent::onMouseMove(double x, double y)
{
  const bool left = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  const bool right = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  const double dx = x - last_x_, dy = y - last_y_;
  last_x_ = x;
  last_y_ = y;
  if (!left && !right) {return;}
  int w, h;
  glfwGetWindowSize(window_, &w, &h);
  const bool shift = glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
  const mjtMouse action = right ?
    (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) : (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V);
  moveCamera(mjv_moveCamera, mj_model_, action, dx / h, dy / h, &scene_, &free_cam_);
}

void MujocoViewComponent::render()
{
  mjrRect viewport{0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  mjv_updateScene(mj_model_, mj_data_, &window_opt_, nullptr, &free_cam_, mjCAT_ALL, &scene_);
  mjr_render(viewport, &scene_, &render_ctx_);
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
    ("time  " + std::to_string(mj_data_->time).substr(0, 6)).c_str(),
    paused_ ? "PAUSED" : "running", &render_ctx_);

  // Camera image at the bottom right (C = next camera / off)
  if (inset_index_ >= 0 && inset_index_ < static_cast<int>(cams_.size())) {
    const Camera & c = cams_[inset_index_];
    const int w = viewport.width / 3;
    const int h = w * camera_height_ / camera_width_;
    const mjrRect inset{viewport.width - w - 10, 10, w, h};
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = c.id;
    mjv_updateScene(mj_model_, mj_data_, &camera_opt_, nullptr, &cam, mjCAT_ALL, &scene_);
    mjr_render(inset, &scene_, &render_ctx_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, inset, c.name.c_str(), "C: next", &render_ctx_);
  }
  glfwSwapBuffers(window_);
}

// ==================== publishers ====================
geometry_msgs::msg::TransformStamped MujocoViewComponent::makeTf(
  const std::string & parent, const std::string & child, const mjtNum * pos, const mjtNum * quat) const
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp_;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = pos[0];
  t.transform.translation.y = pos[1];
  t.transform.translation.z = pos[2];
  t.transform.rotation.w = quat[0];
  t.transform.rotation.x = quat[1];
  t.transform.rotation.y = quat[2];
  t.transform.rotation.z = quat[3];
  return t;
}

/************************************************************************************/
/*--------------------------------- Publish Sensors --------------------------------*/
/************************************************************************************/
void MujocoViewComponent::publishState()
{
  const mjModel * m = mj_model_;
  const mjData * d = mj_data_;
  const mjtNum unit[4] = {1, 0, 0, 0};
  stamp_ = now();

  // TF: world -> every body
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  for (int b = 1; b < m->nbody; ++b) {
    tfs.push_back(makeTf(world_frame_, bodyFrame(b), d->xpos + 3 * b, d->xquat + 4 * b));
  }

  // Joint states
  sensor_msgs::msg::JointState js;
  js.header.stamp = stamp_;
  for (int j : joints_) {
    js.name.push_back(name(mjOBJ_JOINT, j));
    js.position.push_back(d->qpos[m->jnt_qposadr[j]]);
    js.velocity.push_back(d->qvel[m->jnt_dofadr[j]]);
    js.effort.push_back(d->qfrc_actuator[m->jnt_dofadr[j]]);
  }
  joint_pub_->publish(js);

  // IMU (frame = parent body of the IMU site)
  if (imu_pub_) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp_;
    const int s = imu_acc_ >= 0 ? imu_acc_ : (imu_gyro_ >= 0 ? imu_gyro_ : imu_quat_);
    imu.header.frame_id = bodyFrame(m->site_bodyid[m->sensor_objid[s]]);
    if (imu_acc_ >= 0) {
      const mjtNum * a = d->sensordata + m->sensor_adr[imu_acc_];
      imu.linear_acceleration.x = a[0];
      imu.linear_acceleration.y = a[1];
      imu.linear_acceleration.z = a[2];
    }
    if (imu_gyro_ >= 0) {
      const mjtNum * g = d->sensordata + m->sensor_adr[imu_gyro_];
      imu.angular_velocity.x = g[0];
      imu.angular_velocity.y = g[1];
      imu.angular_velocity.z = g[2];
    }
    if (imu_quat_ >= 0) {
      const mjtNum * q = d->sensordata + m->sensor_adr[imu_quat_];
      imu.orientation.w = q[0];
      imu.orientation.x = q[1];
      imu.orientation.y = q[2];
      imu.orientation.z = q[3];
    } else {
      imu.orientation_covariance[0] = -1.0;  // orientation not available
    }
    imu_pub_->publish(imu);
  }

  // LiDAR: rangefinder i looks along the z axis of its site; -1 (no hit) -> +inf
  if (scan_pub_) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp_;
    scan.header.frame_id = lidar_prefix_ + "_frame";
    std::vector<double> angles;
    for (int s : lidar_) {
      const int site = m->sensor_objid[s];
      mjtNum mat[9];
      mju_quat2Mat(mat, m->site_quat + 4 * site);      // site orientation in its body frame
      angles.push_back(std::atan2(mat[5], mat[2]));     // azimuth of the site z axis
      const double r = d->sensordata[m->sensor_adr[s]];
      scan.ranges.push_back(r < 0 ? std::numeric_limits<float>::infinity() : r);
    }
    scan.angle_min = angles.front();
    scan.angle_increment = angles.size() > 1 ? std::remainder(angles[1] - angles[0], 2 * M_PI) : 0.0;
    scan.angle_max = scan.angle_min + scan.angle_increment * (angles.size() - 1);
    scan.range_min = 0.0;
    scan.range_max = lidar_range_max_;
    scan_pub_->publish(scan);

    const int site = m->sensor_objid[lidar_.front()];
    tfs.push_back(makeTf(bodyFrame(m->site_bodyid[site]), scan.header.frame_id, m->site_pos + 3 * site, unit));
  }

  for (auto & [s, pub] : touch_) {
    std_msgs::msg::Float32 msg;
    msg.data = d->sensordata[m->sensor_adr[s]];
    pub->publish(msg);
  }

  // Contact force: frame "<sensor>_frame" at the mean contact point of geom1 (world-aligned),
  // force acting ON geom1 (sensor data = found, Fx, Fy, Fz with the opposite sign)
  for (auto & [s, pub] : contact_) {
    const int geom = m->sensor_objid[s];
    mjtNum point[3] = {0, 0, 0};
    int n = 0;
    for (int i = 0; i < d->ncon; ++i) {
      if (d->contact[i].geom[0] == geom || d->contact[i].geom[1] == geom) {
        mju_addTo3(point, d->contact[i].pos);
        ++n;
      }
    }
    if (n > 0) {
      mju_scl3(point, point, 1.0 / n);
    } else {
      mju_copy3(point, d->geom_xpos + 3 * geom);
    }
    const std::string frame = name(mjOBJ_SENSOR, s) + "_frame";
    tfs.push_back(makeTf(world_frame_, frame, point, unit));

    geometry_msgs::msg::WrenchStamped w;
    w.header.stamp = stamp_;
    w.header.frame_id = frame;
    const mjtNum * v = d->sensordata + m->sensor_adr[s];
    w.wrench.force.x = -v[1];
    w.wrench.force.y = -v[2];
    w.wrench.force.z = -v[3];
    pub->publish(w);
  }

  // Camera optical frames (ROS: z forward, y down = MuJoCo camera rotated 180 deg about x)
  for (const auto & c : cams_) {
    const mjtNum flip[4] = {0, 1, 0, 0};
    mjtNum q[4];
    mju_mulQuat(q, m->cam_quat + 4 * c.id, flip);
    tfs.push_back(makeTf(bodyFrame(m->cam_bodyid[c.id]), c.name + "_optical_frame", m->cam_pos + 3 * c.id, q));
  }
  tf_->sendTransform(tfs);
}

/************************************************************************************/
/*-------------------------------- Publish Cameras  --------------------------------*/
/************************************************************************************/
void MujocoViewComponent::publishCameras()
{
  const mjrRect vp{0, 0, camera_width_, camera_height_};
  const double extent = mj_model_->stat.extent;
  const double znear = mj_model_->vis.map.znear * extent, zfar = mj_model_->vis.map.zfar * extent;

  mjr_setBuffer(mjFB_OFFSCREEN, &render_ctx_);
  for (const auto & c : cams_) {
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = c.id;
    mjv_updateScene(mj_model_, mj_data_, &camera_opt_, nullptr, &cam, mjCAT_ALL, &scene_);
    mjr_render(vp, &scene_, &render_ctx_);
    mjr_readPixels(rgb_buf_.data(), depth_buf_.data(), vp, &render_ctx_);

    sensor_msgs::msg::Image rgb, depth;
    rgb.header.stamp = depth.header.stamp = stamp_;
    rgb.header.frame_id = depth.header.frame_id = c.name + "_optical_frame";
    rgb.height = depth.height = camera_height_;
    rgb.width = depth.width = camera_width_;
    rgb.encoding = "rgb8";
    rgb.step = 3 * camera_width_;
    rgb.data.resize(rgb.step * camera_height_);
    depth.encoding = "32FC1";
    depth.step = 4 * camera_width_;
    depth.data.resize(depth.step * camera_height_);

    for (int r = 0; r < camera_height_; ++r) {  // OpenGL rows are bottom-up
      const int src = camera_height_ - 1 - r;
      std::copy_n(&rgb_buf_[3 * src * camera_width_], 3 * camera_width_, &rgb.data[r * rgb.step]);
      auto * out = reinterpret_cast<float *>(&depth.data[r * depth.step]);
      for (int x = 0; x < camera_width_; ++x) {  // depth buffer [0,1] -> distance [m], NaN = no hit
        const float z = depth_buf_[src * camera_width_ + x];
        out[x] = z >= 1.0f ? std::numeric_limits<float>::quiet_NaN() :
          static_cast<float>(znear / (1.0 - z * (1.0 - znear / zfar)));
      }
    }
    c.rgb->publish(rgb);
    c.depth->publish(depth);

    sensor_msgs::msg::CameraInfo info;
    info.header = rgb.header;
    info.height = camera_height_;
    info.width = camera_width_;
    const double f = 0.5 * camera_height_ / std::tan(0.5 * mj_model_->cam_fovy[c.id] * M_PI / 180.0);
    info.k = {f, 0, 0.5 * camera_width_, 0, f, 0.5 * camera_height_, 0, 0, 1};
    info.p = {f, 0, 0.5 * camera_width_, 0, 0, f, 0.5 * camera_height_, 0, 0, 0, 1, 0};
    c.info->publish(info);
  }
  mjr_setBuffer(mjFB_WINDOW, &render_ctx_);
}

}  // namespace mujoco_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(mujoco_ros2::MujocoViewComponent)