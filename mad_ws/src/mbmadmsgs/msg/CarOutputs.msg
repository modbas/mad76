#
# @brief CarOutputs.msg message type for vision measurement signals
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

uint8 carid               # car identifier
uint8 CARID_UNKNOWN=255   # vision cannot detect carid
float32[2] s              # cartesian center position [ m ]
float32 psi               # yaw angle [ rad ]
float32 prob              # probability of valid measurement [ 0 ... 1 ]
