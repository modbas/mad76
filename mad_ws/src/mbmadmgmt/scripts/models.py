#!/usr/bin/env python3
#
# @brief SQL ORM Models for Race
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

from peewee import Model, SqliteDatabase, IntegerField, FloatField, CharField, BooleanField, DateTimeField, ForeignKeyField
from ament_index_python.packages import get_package_share_directory
import os


# Define the SQLite database connection
data_directory = os.path.join(get_package_share_directory('mbmadmgmt'), 'data')
dbpath = os.path.join(data_directory, 'mad.db')
db = SqliteDatabase(dbpath)

# Driver model
class Driver(Model):
    id = IntegerField(primary_key=True)  # Primary key
    name = CharField(max_length=64)  # Driver name
    robot = BooleanField(default=False)  # Is the driver a robot?

    class Meta:
        database = db  # Link the model to the database

# Race model
class Race(Model):
    id = IntegerField(primary_key=True)  # Primary key
    name = CharField(max_length=64, default="MAD76")
    timestamp = DateTimeField()
    
    def to_dict(self):
        return {
            'id': self.id,
            'name': self.name,
            'timestamp': self.timestamp.isoformat() if self.timestamp else None
        }

    class Meta:
        database = db  # Link the model to the database

# Car model
class Car(Model):
    id = IntegerField(primary_key=True)  # Primary key
    minlaptime = FloatField(default=9999.0)              # Lap time in seconds
    maxavgspeed = FloatField(default=0.0)             # Average speed in km/h
    carid = IntegerField(default=0)                # Car ID
    race = ForeignKeyField(Race, backref='cars', on_delete='CASCADE') # Foreign key to race 
    driver = ForeignKeyField(Driver, backref='cars', on_delete='CASCADE') # Foreign key to driver

    currentlaptime = 0.0
    laptime = 0.0
    avgspeed = 0.0
    lapctr = 0
    crashctr = 0

    # convert object to dictionary
    def to_dict(self):
        return {
            'id': self.id,
            'laptime': self.laptime,
            'avgspeed': self.avgspeed,
            'minlaptime': self.minlaptime,
            'maxavgspeed': self.maxavgspeed,
            'currentlaptime': self.currentlaptime,
            'lapctr': self.lapctr,
            'crashctr': self.crashctr,
            'drivername': self.driver.name if self.driver else None,
        }

    class Meta:
        database = db  # Link the model to the database
        