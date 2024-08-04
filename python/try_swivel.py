################################################################################
# Copyright 2021 E. Kooistra
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################
"""Try swivel pointings

Print InputTube (pointer and angle) and OutputThrust (pointer and angle) as
function of theta (of angle mid_tube) and phi (angle of input_tube).

p_phi = InputTubeAngle(p) is the input tube angle to make thrust vector y = 0.

Usage:
# on command line
> D:
> cd "Mijn documenten\\Openscad\\Swivel"
> python try_swivel.py

# in iPython:
import os
filepath='D:\\Mijn documenten\\Openscad\\Swivel'
os.chdir(filepath)
run try_swivel.py
"""

import numpy as np

c_eps = 0.01


def Rx(angle):
    """Rotate along x-axis from y to z."""
    a = np.radians(angle)
    co, si = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0],
                     [0, co, -si, 0],
                     [0, si, co, 0],
                     [0, 0, 0, 1]])


def Ry(angle):
    """Rotate along y-axis from z to x."""
    a = np.radians(angle)
    co, si = np.cos(a), np.sin(a)
    return np.array([[co, 0, si, 0],
                     [0, 1, 0, 0],
                     [-si, 0, co, 0],
                     [0, 0, 0, 1]])


def Rz(angle):
    """Rotate along z-axis from x to y."""
    a = np.radians(angle)
    co, si = np.cos(a), np.sin(a)
    return np.array([[co, -si, 0, 0],
                     [si, co, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def Tr(x, y, z):
    """Translate by [x, y, z]."""
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])


def SwivelOutputPoint(x_input, x_mid, x_output, phi, theta, alpha):
    """Pointing from origin to swivel output."""
    x = np.array([0, 0, 0, 1])
    return \
        Rx(phi) @ \
        Tr(x_input, 0, 0) @ Ry(alpha) @ Rx(-theta) @ Ry(-alpha) @ \
        Tr(x_mid, 0, 0) @ Ry(-alpha) @ Rx(theta) @ Ry(alpha) @ Tr(x_output, 0, 0) @ x


def InputTubeAngle(p):
    """Input tube angle to make thrust vector y = 0."""
    y = p[1]
    z = p[2]
    return -90 + np.degrees(np.arctan2(z, y))


def OutputThrustAngleY(p):
    """Output thrust angle Y (left, right)."""
    x = p[0]
    y = p[1]
    return np.degrees(np.arctan2(y, x))


def OutputThrustAngleZ(p):
    """Output thrust angle Z (up, down)."""
    x = p[0]
    z = p[2]
    return np.degrees(np.arctan2(z, x))


alpha = 22.5     # Segment angle
x_input = 50     # Length of input segment
x_mid = 80       # Length of middle segment
x_output = 40    # Length of output segment

phi = 0

print('>>> Swivel dimensions:')
print('  alpha    = %5.1f' % alpha)
print('  x_input  = %5d' % x_input)
print('  x_mid    = %5d' % x_mid)
print('  x_output = %5d' % x_output)
print('')

print('>>> InputTube pointer and angle as function of theta')
print('phi = %5.1f:' % phi)
for theta in range(0, 361, 15):
    p = SwivelOutputPoint(x_input, x_mid, x_output, phi, theta, alpha)
    p_phi = InputTubeAngle(p)
    print('  theta = %3d : InputTube pointer = [%5.1f, %5.1f, %5.1f, %3.1f], angle = %6.1f' %
          (theta, p[0], p[1], p[2], p[3], p_phi))
print('')

print('>>> InputTube (pointer and angle) and OutputThrust (pointer and angle) as function of theta and phi:')
step = 30
for theta in range(0, 361, step):
    print('. theta = %5.1f:' % theta)
    print('    phi:          InputTube pointer,  angle,         OutputThrust pointer,  angle, angleY, angleZ')
    for phi in range(0, 361, step):
        p = SwivelOutputPoint(x_input, x_mid, x_output, phi, theta, alpha)
        p_phi = InputTubeAngle(p)
        if p_phi >= 0:
            q_phi = 180-p_phi
        else:
            q_phi = -180-p_phi
        # q = SwivelOutputPoint(x_input, x_mid, x_output, q_phi, theta, alpha)
        q = SwivelOutputPoint(x_input, x_mid, x_output, q_phi, theta, alpha) - \
            SwivelOutputPoint(x_input, x_mid, 0, q_phi, theta, alpha)
        angleY = OutputThrustAngleY(q)
        angleZ = OutputThrustAngleZ(q)
        print('  %5.1f: [%5.1f, %5.1f, %5.1f, %3.1f], %6.1f,   [%5.1f, %5.1f, %5.1f, %3.1f], %6.1f, %6.1f, %6.1f' %
              (phi, p[0], p[1], p[2], p[3], p_phi, q[0], q[1], q[2], q[3], q_phi, angleY, angleY))
