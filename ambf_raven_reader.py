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

ambf_raven_reader.py

date: May 13, 2024
author: Natalie Chalfant, Mai Bui, Sean Fabrega
"""

import pandas as pd


class ambf_raven_reader:

    def __init__(self):
        self.df = None
        self.status = False
        self.row = 0
        self.length = 0

    def get_status(self):
        return self.status

    def stop(self):
        self.df = None
        self.status = False
        self.row = 0
        self.length = 0

    def load_csv(self, filename, type_of_csv):
        self.df = pd.read_csv(filename)
        if type_of_csv == "controller":
            if self.df.shape[1] == 14:
                self.status = True
                self.length = self.df.shape[0]
                print("Successfully loaded ", filename)
                return True
            else:
                return False
        elif type_of_csv == "jpos":
            if self.df.shape[1] == 240:
                self.status = True
                self.length = self.df.shape[0]
                print("Successfully loaded ", filename)
                return True
            else:
                return False

    def read_ci(self):
        if self.row < self.length:
            row = self.df.loc[self.row, :].values.flatten().tolist()
            controller = [row[0:4], row[4:8], row[8:14]]
            self.row += 1
            return controller
        else:
            self.stop()

    def read_jp(self):
        if self.row < self.length:
            row = self.df.loc[self.row, :].values.flatten().tolist()
            jp = [row[1:17]]
            time = [row[0]]
            self.row += 1
            return time, jp
        else:
            self.stop()

