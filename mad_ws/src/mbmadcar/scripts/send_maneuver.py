#!/usr/bin/env python3

#
# @brief send_maneuver.py sends maneuver messages to motion contol
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

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from mbmadmsgs.srv import TrackGetWaypoints
from mbmadmsgs.msg import DriveManeuver


class SendManeuverNode(Node):
  def __init__(self):
    super().__init__('send_maneuver')
    self.carid = -1
    vmax = 0.5
    xref = 0.0
    type = DriveManeuver.TYPE_PATHFOLLOW
    alpha = -1.0
    
    if len(sys.argv) > 1:
      self.carid = int(sys.argv[1])

    if len(sys.argv) > 2:
      vmax = float(sys.argv[2])

    if len(sys.argv) > 3:
      alpha = float(sys.argv[3])
    
    if len(sys.argv) > 4:
      xref = float(sys.argv[4]) 
      type = DriveManeuver.TYPE_PARK

    # register DriveManeuver publisher
    qos = QoSProfile(
      reliability = QoSReliabilityPolicy.RELIABLE,
      #durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
      history = QoSHistoryPolicy.KEEP_LAST,
      depth=1)
    self.maneuverPubs = []
    self.maneuverpassPubs = []
    for carid in range(4):
      self.maneuverPubs.append(
        self.create_publisher(DriveManeuver, f"/mad/car{carid}/maneuver", qos)
      )
      self.maneuverpassPubs.append(
        self.create_publisher(DriveManeuver, f"/mad/car{carid}/maneuverpass", qos)
      )
    client = self.create_client(TrackGetWaypoints, '/mad/get_waypoints')
    while not client.wait_for_service(timeout_sec = 1.0):
      self.get_logger().info('service /mad/get_waypoints is not available, waiting again ...')
    self.maneuver = self.compute_maneuver(client, self.carid, vmax, alpha, type, xref)
    self.maneuverpass = self.compute_maneuver(client, self.carid, vmax, 0.8, type, xref)
    self.timer = self.create_timer(1.0, self.step)

  def compute_maneuver(self, client, carid, vmax, alpha, type, xref):
    request = TrackGetWaypoints.Request(
      dx = 0.01,
      alpha = alpha,
      segment_sequence = [ 0 ]
      # segment_sequence = [ 0, 1, 2, 19, 20 ]
      # segmentSequence = [ 11, 12, 13, 14, 15, 16, 17 ] # get path containing sequence of segments
      # segmentSequence = [ 16, 18, 19, 10, 11, 12, 13, 0, 1 ]
    )
    future = client.call_async(request)
    rclpy.spin_until_future_complete(self, future)
    try:
      result = future.result()
    except Exception as e:
      self.get_logger().error('service call failed %r' % (e,))
    else:
      return DriveManeuver(carid = carid,
        breaks = result.breaks,
        segments = result.segments,
        s1 = result.s1,
        s2 = result.s2,
        spline_coefs1 = result.spline_coefs1,
        spline_coefs2 = result.spline_coefs2,
        periodic = True,
        vmax = vmax, # reference speed
        type = type,
        xref = xref, # target parking position
        lapcount = 0
      )
     
      

  def step(self):
      if self.carid < 0:
        for carid in range(0,4):
          self.maneuver.carid = carid
          self.maneuverpass.carid = carid
          self.maneuverPubs[carid].publish(self.maneuver)
          self.maneuverpassPubs[carid].publish(self.maneuverpass)
      else:
        self.maneuverPubs[self.carid].publish(self.maneuver)
        self.maneuverpassPubs[self.carid].publish(self.maneuverpass)
      #self.timer.cancel() # stop timer


def main():
  # create ROS node
  rclpy.init(args=sys.argv)
  node = SendManeuverNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
