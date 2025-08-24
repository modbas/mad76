#!/usr/bin/env python3

"""
mbmadrclib.py
-------------

MAD76 RCLib for Raspberry Pi GPIO and SPI

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

import spidev
import RPi.GPIO as io

CAR_CNT = 4 # number of cars
POWER_PINS = [ 25, 23, 24, 18 ] # { GPIO25, pin22 ; GPIO23, pin16 ; GPIO24, pin18 ; GPIO18, pin12 }
PEDALS_MAX = 1.0 # maximum pedals value
STEERING_MAX = 1.0 # maximum steering value
SPI_CHANNEL = 0 # SPI channel for communication with the car
SPI_SPEED = 1000000 # SPI speed in Hz
SPI_CMD_PEDALS = 0x11 # command to write pedals
SPI_CMD_STEERING = 0x12 # command to write steering

pedals_data = [ 0x00 ] * CAR_CNT * 2
steering_data = [ 0x00 ] * CAR_CNT * 2

def signal_to_spi_value(value, max_value):
    """Convert a signal value to an SPI value."""
    if value < -max_value or value > max_value:
        raise ValueError("Value must be between {} and {}".format(-max_value, max_value))
    return int((value + max_value) / (2.0 * max_value) * 255.0)

def initialize_spi():
    """
    Initialize the SPI interface.

    Args:
        device (int): SPI device number (default: 0).
        speed (int): SPI speed in Hz (default: 1000000).
    
    Returns:
        spidev.SpiDev: Configured SPI device.
    """
    spi = spidev.SpiDev()
    spi.open(0, SPI_CHANNEL)
    spi.max_speed_hz = SPI_SPEED
    spi.mode = 0b00
    spi.bits_per_word = 8
    spi.lsbfirst = False
    spi.cshigh = False

    for i in range(CAR_CNT):
        pedals_data[2*i] = SPI_CMD_PEDALS
        pedals_data[2*i+1] = signal_to_spi_value(0.0, PEDALS_MAX)
        steering_data[2*i] = SPI_CMD_STEERING
        steering_data[2*i+1] = signal_to_spi_value(0.0, STEERING_MAX)
        
    return spi

def initialize_gpio():
    """Initialize GPIO pins for power control.
    """
    io.setmode(io.BCM)
    for pin in POWER_PINS:
        io.setup(pin, io.OUT)
        io.output(pin, io.LOW)  # Set all power pins to LOW initially

def cleanup_gpio():
    """Clean up GPIO pins.
    """
    io.cleanup()  # Reset all GPIO pins to their default state
    
def switchon_rcpower(carid):
    """Switch on the power for the specified car.

    Args:
        carid (int): Car ID (0 to CAR_CNT-1).
    """
    if carid < 0 or carid >= CAR_CNT:
        raise ValueError("carid must be between 0 and {}".format(CAR_CNT - 1))
    io.output(POWER_PINS[carid], io.HIGH)  # Set the specified power pin to HIGH

def switchoff_rcpower(carid):
    """Switch off the power for the specified car.

    Args:
        carid (int): Car ID (0 to CAR_CNT-1).
    """
    if carid < 0 or carid >= CAR_CNT:
        raise ValueError("carid must be between 0 and {}".format(CAR_CNT - 1))
    io.output(POWER_PINS[carid], io.LOW)  # Set the specified power pin to LOW

def write_pedals(spi, carid, pedals):
    """Write pedals value to the specified car.

    Args:
        spi (spidev.SpiDev): Configured SPI device.
        carid (int): Car ID (0 to CAR_CNT-1).
        pedals (float): Pedals value (-PEDALS_MAX to PEDALS_MAX).
    """
    if carid < 0 or carid >= CAR_CNT:
        raise ValueError("carid must be between 0 and {}".format(CAR_CNT - 1))
    id = CAR_CNT - carid - 1  # Reverse order for SPI communication
    pedals_data[id * 2 + 1] = signal_to_spi_value(pedals, PEDALS_MAX)
    spi.writebytes(pedals_data)

def write_steering(spi, carid, steering):
    """Write steering value to the specified car.
    
    Args:
        spi (spidev.SpiDev): Configured SPI device.
        carid (int): Car ID (0 to CAR_CNT-1).
        steering (float): Steering value (-STEERING_MAX to STEERING_MAX).
    """
    if carid < 0 or carid >= CAR_CNT:
        raise ValueError("carid must be between 0 and {}".format(CAR_CNT - 1))
    id = CAR_CNT - carid - 1  # Reverse order for SPI communication
    steering_data[id * 2 + 1] = signal_to_spi_value(steering, STEERING_MAX)
    spi.writebytes(steering_data)