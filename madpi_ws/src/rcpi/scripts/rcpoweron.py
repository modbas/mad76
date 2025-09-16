#!/usr/bin/env python3

"""
rcpoweron.py
---------

Script to test power on one RC on a Raspberry Pi.

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

CAR_CNT = 4  # Total number of cars in RCs
POWER_PINS = [ 25, 23, 24, 18 ]
PEDALS_MAX = 1.0  # Maximum value for pedals
STEERING_MAX = 1.0  # Maximum value for steering

def usage():
    """Print usage information."""
    print("Usage: python rcpoweron.py <carid>")
    print("This script tests powering on one RC on a Raspberry Pi.")

def command_line_args():
    """Parse command line arguments."""
    # default values
    success = True
    carid = 0 # integer with arbitray wordlength
    if len(sys.argv) < 2:
        success = False
    try:
        carid = int(sys.argv[1])
        if carid < 0 or carid >= CAR_CNT:
            raise ValueError("carid must be between 0 and {}".format(CAR_CNT - 1))        
    except (ValueError, IndexError):
        success = False
    return success, carid



if __name__ == "__main__":
    [ success, carid ] = command_line_args()
    if not success:
        usage()
        sys.exit(1)
    
    # Use GPIO BCM mode
    io.setmode(io.BCM)

    # Configure GPIO as dital output
    io.setup(POWER_PINS[carid], io.OUT)

    # Power on RC 1
    io.output(POWER_PINS[carid], io.HIGH)

    # Cleanup GPIO
    #io.cleanup()
    
    # exit cleanly
    sys.exit(0)
