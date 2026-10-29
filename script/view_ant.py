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

"""Minimal MuJoCo viewer: load the ant sensor model and display it.

Usage (from the repository root or anywhere):
    python scripts/view_ant.py                       # loads models/ant_sensors.xml
    python scripts/view_ant.py path/to/other.xml     # loads another model

Viewer tips:
    F4      : sensor values (IMU, LiDAR, foot touch / contact force, joints)
    Y       : show LiDAR rays (rangefinder visualization)
    F       : show contact forces
    [ / ]   : switch to the front camera "front_cam"
    Control : move the joints with the sliders (right panel)
"""
import sys
from pathlib import Path

import mujoco
import mujoco.viewer

# Default model: <repo>/models/ant_sensors.xml (resolved relative to this file)
DEFAULT_XML = Path(__file__).resolve().parent.parent / "models" / "ant_sensors.xml"


def main():
    xml_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_XML
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Standard interactive viewer (blocks until the window is closed)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
