#
# @brief CarLapcounter.msg message type for lap statistics
#    
# Copyright (C) 2024-2025, Frank Traenkle, Hochschule Heilbronn
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

int32 lapctr                # current lap number
int32 collisionctr          # total number of collisions
int32 offtrackctr           # total number of offtrack events
float32 laptime             # last lap time [ s ]
float32 avgspeed            # average speed of last lap [ m/s ]
