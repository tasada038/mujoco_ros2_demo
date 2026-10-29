#!/usr/bin/env python3

# MIT License

# Copyright (c) 2026 Takumi Asada

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Drive the ant from Python: walk forward, read sensors, stop in front of a wall.

What this sample shows
  1. control : write data.ctrl every step (simple open-loop gait)
  2. sensors : read IMU, LiDAR, foot contact and joints by name with data.sensor()
  3. logic   : use the front LiDAR ray to stop before the wall
  4. camera  : render RGB / depth from "front_cam" (headless mode only)

Usage (from the repository root or anywhere):
    python scripts/run_ant.py                  # viewer, loads models/ant_sensors.xml
    python scripts/run_ant.py --headless       # no window, print sensor values and save camera images
    python scripts/run_ant.py path/to/other.xml --duration 20
"""
import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

# Default model: <repo>/models/ant_sensors.xml (resolved relative to this file)
DEFAULT_XML = Path(__file__).resolve().parent.parent / "models" / "ant_sensors.xml"

# Gait parameters
GAIT_FREQ = 1.5                    # steps per second [Hz]
HIP_AMP = np.radians(25)           # hip swing amplitude [rad]
ANKLE_LIFT = np.radians(35)        # how much a foot is lifted in the swing phase [rad]
HIP_SIGN = [1, -1, 1, -1]          # hip direction for leg 1..4 (front-left, front-right, back-left, back-right)
LEG_PHASE = [0, 0, np.pi, np.pi]   # front legs together, back legs half a cycle later

STOP_DISTANCE = 1.0                # stop when the front LiDAR ray sees something closer than this [m]
                                  # (the front feet reach ~0.7 m ahead of the torso center)
LOG_INTERVAL = 0.5                 # print sensor values every ... [s]


# ----------------------------------------------------------------------------
# Sensors
# ----------------------------------------------------------------------------
def read_sensors(model, data):
    """Collect the sensor values defined in the XML into a dict."""
    s = lambda name: data.sensor(name).data.copy()
    n_lidar = sum(1 for i in range(model.nsensor) if model.sensor(i).name.startswith("lidar"))
    return {
        "imu_acc": s("imu_acc"),                     # (3,) torso frame [m/s^2]
        "imu_gyro": s("imu_gyro"),                   # (3,) torso frame [rad/s]
        "imu_quat": s("imu_quat"),                   # (4,) world frame (w, x, y, z)
        "lidar": np.array([s(f"lidar{i:02d}")[0] for i in range(n_lidar)]),  # (36,) [m], -1 = no hit
        "touch": np.array([s(f"foot{i}_touch")[0] for i in range(1, 5)]),    # (4,) [N]
        "contact": np.array([s(f"foot{i}_contact") for i in range(1, 5)]),   # (4, 4) found, Fx, Fy, Fz
        "joint_pos": np.array([s(f"{j}{i}_pos")[0] for i in range(1, 5) for j in ("hip", "ankle")]),
        "joint_vel": np.array([s(f"{j}{i}_vel")[0] for i in range(1, 5) for j in ("hip", "ankle")]),
    }


# ----------------------------------------------------------------------------
# Control
# ----------------------------------------------------------------------------
def gait_ctrl(model, t):
    """Open-loop gait: hips swing, each foot is lifted while its hip moves forward."""
    ctrl = np.zeros(model.nu)
    phase = 2 * np.pi * GAIT_FREQ * t
    for leg in range(4):
        p = phase + LEG_PHASE[leg]
        ctrl[model.actuator(f"hip{leg + 1}_act").id] = HIP_SIGN[leg] * HIP_AMP * np.sin(p)
        ctrl[model.actuator(f"ankle{leg + 1}_act").id] = -ANKLE_LIFT * max(0.0, np.cos(p))
    return ctrl


class Controller:
    """Walk forward until the front LiDAR ray (lidar00 = torso +x) sees an obstacle."""

    def __init__(self, model):
        self.model = model
        self.stopped = False

    def __call__(self, data, sensors):
        front = sensors["lidar"][0]
        if not self.stopped and 0 < front < STOP_DISTANCE:
            self.stopped = True
            print(f"[run] obstacle at {front:.2f} m -> stop")
        if self.stopped:
            return np.zeros(self.model.nu)   # all joints to 0 = standing pose
        return gait_ctrl(self.model, data.time)


def log(data, sensors, touch_mean):
    p = data.qpos[:3]
    print(f"t={data.time:5.2f}s  pos=({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:.2f})  "
          f"front={sensors['lidar'][0]:5.2f} m  "
          f"touch(mean)=[{', '.join(f'{f:5.1f}' for f in touch_mean)}] N")


# ----------------------------------------------------------------------------
# Camera (offscreen rendering)
# ----------------------------------------------------------------------------
def save_front_camera(model, data, width=640, height=480):
    """Render RGB and depth from "front_cam" and save them."""
    from PIL import Image  # pip install pillow

    renderer = mujoco.Renderer(model, height=height, width=width)
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 0   # do not draw LiDAR rays into the camera image

    renderer.update_scene(data, camera="front_cam", scene_option=opt)
    rgb = renderer.render()                       # (H, W, 3) uint8
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera="front_cam", scene_option=opt)
    depth = renderer.render()                     # (H, W) float32 [m]
    renderer.close()

    Image.fromarray(rgb).save("front_rgb.png")
    np.save("front_depth.npy", depth)
    print(f"[run] saved front_rgb.png and front_depth.npy "
          f"(depth at image center = {depth[height // 2, width // 2]:.2f} m)")


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("xml", nargs="?", default=str(DEFAULT_XML))
    ap.add_argument("--headless", action="store_true", help="run without a window")
    ap.add_argument("--duration", type=float, default=15.0, help="simulation time [s]")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    controller = Controller(model)
    next_log = LOG_INTERVAL
    touch_sum, n_sum = np.zeros(4), 0

    def step():
        nonlocal next_log, touch_sum, n_sum
        sensors = read_sensors(model, data)
        data.ctrl[:] = controller(data, sensors)
        mujoco.mj_step(model, data)

        # Feet touch the ground only part of each step: log the mean over the interval
        touch_sum += sensors["touch"]
        n_sum += 1
        if data.time >= next_log:
            log(data, sensors, touch_sum / n_sum)
            touch_sum, n_sum = np.zeros(4), 0
            next_log += LOG_INTERVAL

    if args.headless:
        while data.time < args.duration:
            step()
        save_front_camera(model, data)
        return

    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)  # initial view from the XML
        while viewer.is_running() and data.time < args.duration:
            start = time.time()
            step()
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - start)))  # real time


if __name__ == "__main__":
    main()
