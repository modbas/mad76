#!/usr/bin/env python3

"""
rctestrc1.py
---------

Script to test switch RC1.

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

import time
import sys
import RPi.GPIO as io

CAR_CNT = 4 # number of cars
POWER_PIN = 25

if __name__ == "__main__":
    # Use GPIO BCM mode
    io.setmode(io.BCM)

    # Configure GPIO as dital output
    io.setup(POWER_PIN, io.OUT)

    # Power on RC 1
    io.output(POWER_PIN, io.HIGH)
    time.sleep(5)

    # Power off RC 1
    io.output(POWER_PIN, io.LOW)

    # Cleanup GPIO
    io.cleanup()
    
    # exit cleanly
    sys.exit(0)