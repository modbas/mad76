#!/usr/bin/env python3
#
# @brief ROS2 node TrackNode for detecting MAD76 board contours and /mad/get_waypoints service
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
import rclpy.parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from mbmadmsgs.srv import VisionTransformPoints, TrackGetWaypoints
from BoardVision import BoardVision
from ament_index_python.packages import get_package_share_directory
import numpy as np
#from sensor_msgs.msg import Image
import cv_bridge

class TrackNode(Node):
    def __init__(self):
        super().__init__('tracknode', namespace='/mad')

        # parameters for curbs
        self.declare_parameter('sl1', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('sl2', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('sr1', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('sr2', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.processImageMessage = False
        sl1 = self.get_parameter_or('sl1', rclpy.Parameter('sl1', rclpy.Parameter.Type.DOUBLE_ARRAY, [])).value
        sl2 = self.get_parameter_or('sl1', rclpy.Parameter('sl2', rclpy.Parameter.Type.DOUBLE_ARRAY, [])).value
        sr1 = self.get_parameter_or('sl1', rclpy.Parameter('sr1', rclpy.Parameter.Type.DOUBLE_ARRAY, [])).value
        sr2 = self.get_parameter_or('sl1', rclpy.Parameter('sr2', rclpy.Parameter.Type.DOUBLE_ARRAY, [])).value
        if not sl1 or not sl2 or not sr1 or not sr2:
            self.get_logger().info('no curb parametes found, waiting for image')
            self.processImageMessage = True
            # transform pointts client
            self.clientTransform = self.create_client(VisionTransformPoints, '/mad/vision/transform_points')        
            while not self.clientTransform.wait_for_service(timeout_sec = 1.0):
                self.get_logger().info('service /mad/vision/transform_points is not available, waiting again ...')
        else:
            self.get_logger().info('curb parametes read from parameter server')

        
        qos = QoSProfile(
          reliability = QoSReliabilityPolicy.BEST_EFFORT,
          durability = QoSDurabilityPolicy.VOLATILE,
          history = QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        qosReliable = QoSProfile(
          reliability = QoSReliabilityPolicy.RELIABLE,
          history = QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        
        # waypoint service
        self.srvWaypoints = self.create_service(TrackGetWaypoints, '/mad/get_waypoints', self.getWaypoints)
        # subscribe to vision image topic
        self.subImg = self.create_subscription(Image, '/mad/vision/image', self.imgCallback, qos)
        # publish track markers for rviz
        self.pubTrack = self.create_publisher(MarkerArray, 'track', qosReliable)

        # cyclic time
        self.create_timer(1.0, self.step)

    def imgCallback(self, msg):
        if self.processImageMessage:
            self.get_logger().info('image received')
            # convert image to OpenCV format
            bridge = cv_bridge.CvBridge()
            image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # detect contours and finish line
            vision = BoardVision(image)
            _sc, sl, sr = vision.computeWaypoints()
            if _sc is None:
                self.get_logger().error(vision.errtxt)
                return
            self.get_logger().info('computeWaypoints done')            
            self.__transformWaypoints(sl, self.__transformFutureCallbackSl)
            self.__transformWaypoints(sr, self.__transformFutureCallbackSr)
            self.processImageMessage = False

    def getWaypoints(self, request, response):    
        self.get_logger().info('getWaypoints service called')
        sl = np.array([ self.get_parameter('sl1').value, self.get_parameter('sl2').value ]).T 
        sr = np.array([ self.get_parameter('sr1').value, self.get_parameter('sr2').value ]).T
        if len(sl) > 1 and len(sr) > 1:
            s = np.add(sr * (1.0 - request.alpha), sl * request.alpha)
            x = [ 0 ]
            for i in range(len(s)-1):
                x.append(np.sqrt((s[i+1,0] - s[i,0])**2 + (s[i+1,1] - s[i,1])**2) + x[i])
            response.s1 = s[:,0].tolist()
            response.s2 = s[:,1].tolist()
            response.breaks = x
            response.x_end = x[-1]
        return response
    
    def step(self):
        markerArray = MarkerArray()
        sl1 = self.get_parameter('sl1').value
        sl2 = self.get_parameter('sl2').value
        sl = np.array([ sl1, sl2 ]).T
        if len(sl) > 0:
            markerArray.markers.append(self.__createSplineMarker(sl, [0.0, 1.0, 0.0], 0))
        sr1 = self.get_parameter('sr1').value
        sr2 = self.get_parameter('sr2').value
        sr = np.array([ sr1, sr2 ]).T
        if len(sr) > 0:
            markerArray.markers.append(self.__createSplineMarker(sr, [1.0, 0.0, 0.0], 1))
        self.pubTrack.publish(markerArray)        

    
    def __transformWaypoints(self, sboard, callback):
        request = VisionTransformPoints.Request()
        request.s1 = sboard[:,0].tolist()
        request.s2 = sboard[:,1].tolist()
        future = self.clientTransform.call_async(request)
        future.add_done_callback(callback)

    def __transformFutureCallbackSl(self, future):
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self.get_logger().error('service call failed ' + str(e))
            else:
                sl1 = rclpy.parameter.Parameter('sl1', rclpy.Parameter.Type.DOUBLE_ARRAY, result.s1)
                sl2 = rclpy.parameter.Parameter('sl2', rclpy.Parameter.Type.DOUBLE_ARRAY, result.s2)
                self.set_parameters([sl1, sl2])
                self.get_logger().info('transformed left curb waypoints')
        else:
            self.get_logger().error('service call timeout')
        
    def __transformFutureCallbackSr(self, future):
        if future.done():
            try:
                result = future.result()
            except Exception as e:
                self.get_logger().error('service call failed ' + str(e))
            else:
                sr1 = rclpy.parameter.Parameter('sr1', rclpy.Parameter.Type.DOUBLE_ARRAY, result.s1)
                sr2 = rclpy.parameter.Parameter('sr2', rclpy.Parameter.Type.DOUBLE_ARRAY, result.s2)
                self.set_parameters([sr1, sr2])
                self.get_logger().info('transformed right curb waypoints')
        else:
            self.get_logger().error('service call timeout')    

    def __createSplineMarker(self, s, color, id):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'map'
        marker.id = id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        for i in range(len(s)):
            p = Point()
            p.x = s[i,0]
            p.y = s[i,1]
            p.z = 0.0
            marker.points.append(p)
        return marker
        

def main():
    # create ROS node
    rclpy.init(args=sys.argv)
    node = TrackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
