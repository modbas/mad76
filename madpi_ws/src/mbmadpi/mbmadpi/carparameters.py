#!/usr/bin/env python3

"""
carparameters.py
----------------

MAD76 car parameters

Copyright (C) 2025, Frank Traenkle, Hochschule Heilbronn
 
This file is part of MAD.
MAD is free software: you can redistribute it and/or modify it under the terms 
of the GNU General Public License as published by the Free Software Foundation,
either version 3 of the License, or (at your option) any later version.
MAD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with MAD.
If not, see <https://www.gnu.org/licenses/>.
"""

P_UN_MAX = 0.2 # maximum normalized pedals signal [ 1 ]
P_DELTAN_MAX = 0.93 # maximum normalized steering signal [ 1 ]

TRACK_SIZE = [ -0.1, 0.82, 0.0, 0.5 ] # track size in [ m ]: [ x_min, x_max, y_min, y_max ]
SAFETY_BOUNDARY = 100e-3 # safety boundary in [ m ]: distance to track boundary

JOY_PEDALSAXIS = 1 # joystick axis for pedals
JOY_STEERINGAXIS = 2 # joystick axis for steering
JOY_BUTTON_A = 0 # joystick button for A (override safety halt)
