import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot
from morse.robots.fakerobot import FakeRobot
from morse.middleware.ros_request_manager import ros_service
from morse.core import blenderapi

from std_srvs.srv import Trigger, TriggerResponse
from auto_smart_factory.srv import SetConveyorSpeed
from auto_smart_factory.srv import RotateTable

class RotatingTable(FakeRobot):

    _name = 'Rotating Table'
    
    def __init__(self, obj, parent=None):
        FakeRobot.__init__(self, obj, parent)

    @ros_service(type=SetConveyorSpeed, name='set_speed')
    def set_speed(self, speed):
        self.bge_object["speed"] = speed
        return True

    @ros_service(type=RotateTable, name='rotate')
    def rotate_table(self, rotation):
        if (rotation < -360 or rotation > 360):
            return False
        self.bge_object["table_rotation"] = rotation
        self.bge_object["rotate"] = True
        return True

    @ros_service(type=Trigger, name='switch_on_off')
    def switch_conveyor_on_off(self):
        if self.bge_object["rotate_barrels"]:
            self.bge_object["rotate_barrels"] = False
            return (True, "Turned off")
        else:
            self.bge_object["rotate_barrels"] = True
            return (True, "Running")

