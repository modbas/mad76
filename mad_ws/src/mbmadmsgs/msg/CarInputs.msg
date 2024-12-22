#
# @brief CarInputs.msg message type for car control signals
#    
# Copyright (C) 2024, Frank Traenkle, Hochschule Heilbronn
#  
# This file is part of MAD.
# MAD is free software: you can redistribute it and/or modify it under the terms 
# of the GNU General Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version.
# MAD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY 
# without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
# You should have received a copy of the GNU General Public License along with MAD.
# If not, see <https://www.gnu.org/licenses/>.
#

mbsafemsgs/CheckpointSequence cpseq
uint8 carid              # virtual car id
uint8 opmode             # Operation Mode FSM state
uint8 OPMODE_NORMAL = 0
uint8 OPMODE_SAFETYHALT = 1
uint8 OPMODE_DEGRADED = 2
uint8 cmd                # command to car control
uint8 CMD_HALT = 0       # car halt
uint8 CMD_FORWARD = 1    # low to high speed forward drive
uint8 CMD_REVERSE = 2    # low to high speed reverse drive
uint8 CMD_SLOW = 3       # low speed forward/reverse drive
uint8 CMD_CHARGE = 4     # battery charging
float32 pedals           # motor control [ -1; 1 ]
float32 steering         # steering control [ -1; 1 ]
