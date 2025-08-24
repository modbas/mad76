#!/usr/bin/env python3

"""
rcnode.py
---------

ROS2 Node to remotely control MAD76 cars

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
try:
    import mbmadpi.mbmadrclib as rc
except ImportError:
    import mbmadrclib as rc
try:
    import mbmadpi.carparameters as p
except ImportError:
    import carparameters as p

class RcNode(rclpy.node.Node):
    """
    RcNode is a ROS2 Node to remotely control MAD76 cars.
    """
    
    def __init__(self):
        """
        RcNode constructor."""
        super().__init__('rcnode', namespace='/mad')

    def init(self):
        """ Initialize the RcNode.
        
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        self.carid = 0
        self.spi = rc.initialize_spi()
        if not self.spi:
            self.get_logger().info("Failed to initialize SPI.")
            return False
        
        # initialize GPIO
        rc.initialize_gpio()

        # switch on power for the specified car
        rc.switchon_rcpower(self.carid)

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
        self.sub_carinputs = self.create_subscription(
            mbmadmsgs.msg.CarInputs,
            f'/mad/car{self.carid}/carinputs',
            self.carinputs_callback,
            qosBestEffort
        )
        
        return True
        
        
    def spin(self):
        """ Spin the RcNode to process incoming messages."""
        rclpy.spin(self.node)

    def destroy(self):
        """ Clean up resources and shutdown the RcNode."""
        self.destroy_node()
        
    def carinputs_callback(self, msg):
        """ Callback function for car inputs messages.

        Args:
            msg (mbmadmsgs.msg.CarInputs): The CarInputs message containing carid, pedals, and steering.
        """
        #self.get_logger().info(f'CarInputs msg received: carid={msg.carid}, pedals={msg.pedals}, steering={msg.steering}')

        # saturate pedals
        pedals = msg.pedals
        if pedals > p.P_UN_MAX:
            pedals = p.P_UN_MAX
        elif pedals < -p.P_UN_MAX:
            pedals = -p.P_UN_MAX
        
        # saturate steering
        steering = msg.steering
        if steering > p.P_DELTAN_MAX:
            steering = p.P_DELTAN_MAX
        elif steering < -p.P_DELTAN_MAX:
            steering = -p.P_DELTAN_MAX

        # SPI output
        rc.write_pedals(self.spi, self.carid, pedals)
        rc.write_steering(self.spi, self.carid, steering)
        
def main():
    """ Main function to initialize and run the RcNode."""
    ret = 0
    rclpy.init(args=sys.argv)
    node = RcNode()
    if not node.init():
        node.get_logger().info("Initialization failed, shutting down.")
        ret = 1
    else:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
