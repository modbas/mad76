#
# @brief DriveManeuver.msg message type for motion control input
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
uint8 carid                                   # id of the car
float32[] breaks                              # path spline breaks
uint32[] segments                             # segment id of each break
float32[] s1                                  # s1 coordinates
float32[] s2                                  # s2 coordinates
float32[] spline_coefs1                       # path spline coefficients of s1 coordinate
float32[] spline_coefs2                       # path spline coefficients of s2 coordinate
bool periodic                                 # spline is periodic or non-periodic
float32 vmax                                  # maximum speed
uint8 type                                    # type of maneuver
uint8 TYPE_HALT = 0
uint8 TYPE_PARK = 1
uint8 TYPE_CHARGE = 2
uint8 TYPE_PATHFOLLOW = 3
uint8 TYPE_LANECHANGE = 4
uint8 TYPE_BREAKUPLANECHANGE = 5
float32 xref                                  # target arc length in case of PARK
uint32 lapcount                               # number of laps in case of PARK
