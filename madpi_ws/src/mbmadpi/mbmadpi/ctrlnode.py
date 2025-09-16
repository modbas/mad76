#!/usr/bin/env python3

"""
ctrlnode.py
-----------

ROS2 Node for joystick control of MAD76 cars

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

import sys
import rclpy
import rclpy.node
import mbmadmsgs.msg
import sensor_msgs.msg
try:
    import mbmadpi.carparameters as p
except ImportError:
    import carparameters as p


class CtrlNode(rclpy.node.Node):
    """
    CtrlNode is a ROS2 Node for joystick control of MAD76 cars.
    """
    
    def __init__(self):
        """
        CtrlNode constructor."""
        super().__init__('ctrlnode', namespace='/mad')

    def init(self):
        """ Initialize the CtrlNode.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        self.carid = 0
        
        qosBestEffort = rclpy.qos.QoSProfile(
          reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
          durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
          history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        qosReliable = rclpy.qos.QoSProfile(
          reliability = rclpy.qos.QoSReliabilityPolicy.RELIABLE,
          durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE,
          history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        self.pub_carinputs = self.create_publisher(
            mbmadmsgs.msg.CarInputs,
            f'/mad/car{self.carid}/carinputs',
            qosBestEffort
        )
        self.sub_joy = self.create_subscription(
            sensor_msgs.msg.Joy,
            f'/joy',
            self.joy_callback,
            qosReliable
        )

        return True
        
        
    def spin(self):
        """ Spin the RcNode to process incoming messages."""
        rclpy.spin(self.node)

    def destroy(self):
        """ Clean up resources and shutdown the RcNode."""
        self.destroy_node()
        
    def joy_callback(self, msg):
        """ Callback function for joystick messages.

        Args:
            msg (sensor_msgs.msg.Joy): Joystick message containing control and button inputs.
        """
        pedals = msg.axes[p.JOY_PEDALSAXIS] * p.P_UN_MAX # normalized pedals signal
        steering = msg.axes[p.JOY_STEERINGAXIS] * p.P_DELTAN_MAX # normalized steering signal
        carinputs_msg = mbmadmsgs.msg.CarInputs()
        carinputs_msg.carid = self.carid
        carinputs_msg.pedals = pedals
        carinputs_msg.steering = steering
        self.pub_carinputs.publish(carinputs_msg)
                
def main():
    """ Main function to initialize and run the RcNode."""
    ret = 0
    rclpy.init(args=sys.argv)
    node = CtrlNode()
    if not node.init():
        node.get_logger().info("Initialization failed, shutting down.")
        ret = 1
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Ctrl-C received, shutting down.")
        finally:
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
