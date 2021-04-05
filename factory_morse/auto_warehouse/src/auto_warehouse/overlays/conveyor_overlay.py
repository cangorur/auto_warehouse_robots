from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status
import time

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float32
from auto_smart_factory.srv import SetConveyorSpeed


class ConveyorControlOverlay(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)
        self.speed = 0.1

    @ros_service(type=SetConveyorSpeed, name='set_speed')
    def set_speed(self, speed):
        self.speed = speed
        return SetConveyorSpeedResponse(True)

    @ros_service(type=Trigger, name='switch_on_off')
    def switch_on_off(self):
        if self.overlaid_object.is_on:
            self.overlaid_object.set_speed(0.0)
        else:
            self.overlaid_object.set_speed(self.speed) # set_speed service of conv belt automatically sets is_on variable
        return TriggerResponse(self.overlaid_object.is_on, 'Is Conveyor Belt Running?: ' + str(self.overlaid_object.is_on))
