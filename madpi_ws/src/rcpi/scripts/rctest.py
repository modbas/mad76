#!/usr/bin/env python3

"""
rctest.py
---------

Script to test the GPIO pins on a Raspberry Pi.

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
import signal
import mbmadrclib as rc

# This script is used to test the GPIO pins on a Raspberry Pi.


def signal_handler(signal, frame):
    """Handle the signal to clean up GPIO on exit."""
    rc.cleanup_gpio()
    sys.exit(0)

def usage():
    """Print usage information."""
    print("Usage: python rctest.py <carid> <pedals> <delta>")
    print("This script tests the GPIO pins on a Raspberry Pi.")
    
def command_line_args():
    """Parse command line arguments."""
    # default values
    success = True
    carid = 0 # integer with arbitray wordlength
    pedals = 0.0 # 64bit float
    steering = 0.0
    if len(sys.argv) < 3:
        success = False
    try:
        carid = int(sys.argv[1])
        if carid < 0 or carid >= rc.CAR_CNT:
            raise ValueError("carid must be between 0 and {}".format(rc.CAR_CNT - 1))
        pedals = float(sys.argv[2])
        if pedals < -rc.PEDALS_MAX or pedals > rc.PEDALS_MAX:
            raise ValueError("pedals must be between {} and {}".format(-rc.PEDALS_MAX, rc.PEDALS_MAX))
        steering = float(sys.argv[3])
        if steering < -rc.STEERING_MAX or steering > rc.STEERING_MAX:
            raise ValueError("steering must be between {} and {}".format(-rc.STEERING_MAX, rc.STEERING_MAX))        
    except (ValueError, IndexError):
        success = False
    return success, carid, pedals, steering



if __name__ == "__main__":
    [ success, carid, pedals, steering ] = command_line_args()
    if not success:
        usage()
        sys.exit(1)
    
    # initialize signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # initialize SPI
    spi = rc.initialize_spi()
    if not spi:
        print("Failed to initialize SPI.")
        sys.exit(1)
    
    # initialize GPIO
    rc.initialize_gpio()

    # switch on power for the specified car
    rc.switchon_rcpower(carid)

    # wait for a short time to ensure power is stable
    time.sleep(1)
    
    # write pedals
    rc.write_pedals(spi, carid, pedals)
    
    # # write pedals
    rc.write_steering(spi, carid, steering)
    
    # exit cleanly
    sys.exit(0)