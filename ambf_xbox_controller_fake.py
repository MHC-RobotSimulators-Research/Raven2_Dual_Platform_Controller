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

ambf_xbox_controller_fake.py

date: May 13, 2024
author: Natalie Chalfant, Mai Bui, Sean Fabrega
"""

import math
import time


class ambf_xbox_controller_fake:

    def __init__(self):
        self.counter = 1

    def read(self):
        # print(self.counter)
        if 1000 <= self.counter:
            self.counter = 1

        lx = math.cos(2 * math.pi * (1000 / self.counter))
        ly = math.sin(2 * math.pi * (1000 / self.counter))
        lt = math.cos(math.pi * (1000 / self.counter))
        lb = 0

        rx = math.cos(2 * math.pi * (1000 / self.counter))
        ry = math.sin(2 * math.pi * (1000 / self.counter))
        rt = math.cos(math.pi * (1000 / self.counter))
        rb = 0

        self.counter += 1

        return [[lx, ly, lt, lb], [rx, ry, rt, rb], [0, 0, 0, 0, 0, 0]]

    def rumble(self, left, right, time):
        pass
