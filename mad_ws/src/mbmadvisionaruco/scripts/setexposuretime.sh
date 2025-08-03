#!/usr/bin/env bash
#
# @brief Workaround for setting exposure time of camera_ros (not working in new versions)
#    
# Copyright (C) 2025, Frank Traenkle, Hochschule Heilbronn
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

# wait for the camera node to start
until ros2 topic list | grep -q "/mad/camera/image_raw"; do
  echo "Waiting for camera node to start..."
  sleep 1
done
ros2 param set /mad/camera ExposureTime 2000
exit 0
