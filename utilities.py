"""
Raven II Dual Platform Controller: control software for the Raven II robot. Copyright © 2023-2024 Yun-Hsuan Su,
Natalie Chalfant, Mai Bui, Sean Fabrega, and the Mount Holyoke Intelligent Medical Robotics Laboratory.

This file is a part of Raven II Dual Platform Controller.

Raven II Dual Platform Controller is free software: you can redistribute it and/or modify it under the terms of the
GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

Raven II Dual Platform Controller is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with Raven II Dual Platform Controller.
If not, see <http://www.gnu.org/licenses/>.

utilities.py

date: May 13, 2024
author: Natalie Chalfant, Mai Bui, Sean Fabrega
"""

from PyKDL import Vector, Rotation, Frame, dot
import numpy as np
import math
import sys

PI = np.pi
PI_2 = np.pi/2

eps = sys.float_info.epsilon


# The up vector is useful to define angles > PI. Since otherwise
# this method will only report angles <= PI.
def get_angle(vec_a, vec_b, up_vector=None):
    vec_a.Normalize()
    vec_b.Normalize()
    cross_ab = vec_a * vec_b
    vdot = dot(vec_a, vec_b)
    # print('VDOT', vdot, vec_a, vec_b)
    # Check if the vectors are in the same direction
    if 1.0 - vdot < 0.000001:
        angle = 0.0
        # Or in the opposite direction
    elif 1.0 + vdot < 0.000001:
        angle = np.pi
    else:
        angle = math.acos(vdot)

    if up_vector is not None:
        same_dir = np.sign(dot(cross_ab, up_vector))
        if same_dir < 0.0:
            angle = -angle

    return angle


def round_mat(mat, rows, cols, precision=4):
    for i in range(0, rows):
        for j in range(0, cols):
            mat[i, j] = round(mat[i, j], precision)
    return mat


def round_vec(vec, precision=4):
    for i in range(3):
        vec[i] = round(vec[i], precision)
    return vec


def round_transform(mat, precision=4):
    return round_mat(mat, 4, 4, precision)


def convert_frame_to_mat(frame):
    np_mat = np.mat([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)
    for i in range(3):
        for j in range(3):
            np_mat[i, j] = frame.M[(i, j)]

    for i in range(3):
        np_mat[i, 3] = frame.p[i]

    return np_mat


def convert_mat_to_frame(mat):
    frame = Frame(Rotation.RPY(0, 0, 0), Vector(0, 0, 0))
    for i in range(3):
        for j in range(3):
            frame[(i, j)] = mat[i, j]

    for i in range(3):
        frame.p[i] = mat[i, 3]

    return frame

def get_Origin(T_matrix):
    origin = T_matrix[:3,3]
    return origin

def set_Origin(T_matrix, x, y, z):
    T_matrix[0,3] = x
    T_matrix[1,3] = y
    T_matrix[2,3] = z

def get_Basis(T_matrix):
    basis = np.zeros((3,3))
    basis = T_matrix[0:3,0:3]
    return basis

def normalize(numpyVec3):
    return numpyVec3 / np.linalg.norm(numpyVec3) #why are we getting zeroes for the z coord?
