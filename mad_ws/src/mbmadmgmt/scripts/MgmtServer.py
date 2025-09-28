#!/usr/bin/env python3
#
# @brief Webserver for race management
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

import sys
import threading
import flask
from datetime import datetime

from flask import app
import rclpy
from rclpy.node import Node
import rclpy.parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from mbmadmsgs.msg import CtrlReference
from mbmadmsgs.srv import ResetLapcounter
from models import db, Race, Driver, Car


class MgmtServerNode(Node):
    def __init__(self):
        super().__init__('mgmtservernode', namespace='/mad')

        # SQL db is in data directory
        db.connect()

        # create tables if not exist
        db.create_tables([Race, Driver, Car], safe=True)

        # get race with highest id
        self.race = Race.select().order_by(Race.id.desc()).first()
        if not self.race:
            # create first race
            self._createrace("MAD76")
            
        self.driver = None
        self.car = None
                
        qos = QoSProfile(
          reliability = QoSReliabilityPolicy.BEST_EFFORT,
          durability = QoSDurabilityPolicy.VOLATILE,
          history = QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        # subscribe ctrlreference topic
        self.subCtrlReference = self.create_subscription(CtrlReference, '/mad/car0/ctrlreference', self._ctrlReferenceCallback, qos)
        self.clientResetLapcounter= self.create_client(ResetLapcounter, '/mad/car0/reset_lapcounter')
        while not self.clientResetLapcounter.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service /mad/car0/reset_lapcounter is not available, waiting again ...')

    def create_app(self):
        app = flask.Flask(__name__)

        # create html space
        @app.route('/')
        def index():
            return flask.render_template('index.html')


        @app.route('/mad.css')
        def mad_css():
            return flask.send_from_directory('static', 'mad.css')

        # get race data
        @app.route('/getracedata', methods=['GET'])
        def getracedata():
            if self.race is None:
                return flask.jsonify({"error": "No race data available"}), 404
            else:
                return flask.jsonify(self.race.to_dict()), 200

        # get car lap data
        @app.route('/getlapdata', methods=['GET'])
        def getlapdata():
            if self.car is None:
                return flask.jsonify({"error": "No car data available"}), 404
            else:
                return flask.jsonify(self.car.to_dict()), 200

        # create new race
        @app.route('/createrace', methods=['POST'])
        def createrace():
            race_name = flask.request.json.get('name')
            if not race_name:
                return flask.jsonify({"error": "Missing race name"}), 400
            else:
                self._createrace(race_name)
                self.driver = None
                self.car = None
            return flask.jsonify({}), 200

        # create new driver
        @app.route('/createdriver', methods=['POST'])
        def createdriver():
            # get driver name from form data
            driver_name = flask.request.json.get('name')
            if not driver_name:
                return flask.jsonify({"error": "Missing driver name"}), 400
            else:
                if self.clientResetLapcounter is None:
                    return flask.jsonify({}), 200
                else:
                    request = ResetLapcounter.Request()
                    future = self.clientResetLapcounter.call_async(request)
                    rclpy.spin_until_future_complete(self, future)
                    try:
                        result = future.result()
                        self.driver = Driver.create(name=driver_name, robot=False)
                        self.car = Car.create(race=self.race.id, driver=self.driver.id, carid=0)                
                        return flask.jsonify({}), 200
                    except Exception as e:
                        self.get_logger().error('service call failed %r' % (e,))
                        return flask.jsonify({"error": "Service call /mad/car0/reset_lapcounter failed"}), 400

        # get ranking
        @app.route('/getranking', methods=['GET'])
        def getranking():
            selfcarid = -1 # no current driver registered (HTML Window onLoad)
            if self.car is not None:
                selfcarid = self.car.id
            # sort cars by lap time
            cars = Car.select().where(Car.race == self.race.id).order_by(Car.minlaptime.asc())
            ranking = [{"active": (car.id == selfcarid), "driver": car.driver.name, "laptime": car.minlaptime, "avgspeed": car.maxavgspeed } for car in cars]
            return flask.jsonify(ranking), 200

        return app
    
    def _createrace(self, name):
        now = self.get_clock().now()
        seconds, nanoseconds = now.seconds_nanoseconds()
        timestamp = datetime.fromtimestamp(seconds + nanoseconds * 1e-9)
        self.race = Race.create(timestamp=timestamp, name=name)

    def _ctrlReferenceCallback(self, msg):
        if self.car is not None:
            lapctrkm1 = self.car.lapctr            
            self.car.lapctr = msg.lapctr
            self.car.currentlaptime = msg.currentlaptime
            self.car.crashctr = msg.crashctr                
            if lapctrkm1 != self.car.lapctr: # lap finished
                self.car.lapctr = msg.lapctr
                self.car.laptime = msg.laptime
                self.car.avgspeed = msg.avgspeed
                if self.car.lapctr > 1 and self.car.laptime < self.car.minlaptime:
                    self.car.minlaptime = self.car.laptime
                    self.car.maxavgspeed = self.car.avgspeed
                # update database
                self.car.save()
            
def ros_spin(node):
    rclpy.spin(node)

def main():
    # create ROS node
    rclpy.init(args=sys.argv)
    node = MgmtServerNode()

    # run ROS as thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,))
    ros_thread.start()

    # run flask webserver
    app = node.create_app()
    app.run(host='0.0.0.0', port=8082)    

    # Wait for the ROS thread to finish
    ros_thread.join()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
