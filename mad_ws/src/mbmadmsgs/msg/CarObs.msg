#
# @brief CarObs.msg message type for object tracking for AI observation space
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

float32[2] s                # cartesian rear axle center position [ m ]
float32 psi                 # yaw angle [ rad ]
float32 beta                # slip angle [ rad ]
float32 v                   # speed [ m/s ]
float32 cxe                 # total arc length of center line [ m ]
float32 cx                  # position on center line [ m ]
float32 cxd                 # speed on center line [ m/s ]
float32[2] cs               # cartesian position on center line [ m ]
float32 cey                 # deviation from center line [ m ]
float32 cepsi               # yaw deviation from center line [ rad ]
float32 ckappa              # curvature of center line [ 1/m ]
float32 rey                 # distance to right curb for crash detection [ m ]
float32 ley                 # distance to left curb for crash detection [ m ]   
float32 prob                # probability of valid measurement [ 0 ... 1 ]
