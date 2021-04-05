import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot
from morse.core.services import service
from morse.middleware.ros_request_manager import ros_service
from morse.core import blenderapi

import fnmatch
import GameLogic
import time
import bpy
import bge

from std_srvs.srv import Trigger, TriggerResponse


class Conveyor(morse.core.robot.Robot):
    """ 
    Class definition for the conveyor robot.
    """
    _name = 'conveyor robot'

    def __init__(self, obj, parent=None):
        """ Constructor method

        Receives the reference to the Blender object.
        Optionally it gets the name of the object's parent,
        but that information is not currently used for a robot.
        """

        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)
        self.is_on = False

        # Do here robot specific initializations
        logger.info('Component initialized')

    def default_action(self):
        """ Main loop of the robot
        """

        # This is usually not used (responsibility of the actuators
        # and sensors). But you can add here robot-level actions.
        pass

    @service
    def set_speed(self, rotation):
        self.bge_object["rotation"] = rotation
        if rotation == 0.0:
            self.is_on = False
        else:
            self.is_on = True

