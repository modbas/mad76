#
# @brief CarReference.msg message type for RViz display of car reference signals
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
uint8 carid
float32[2] s     # cartesian center position [ m ]
float32 psi      # yaw angle [ rad ]
float32 kappa    # curvature [ 1/m ]
float32 v        # center point velocity [ m/s ]
float32 x        # driven arc length of center [ m ]
