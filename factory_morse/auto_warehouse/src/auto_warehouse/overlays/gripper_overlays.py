from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

from std_srvs.srv import Trigger, TriggerResponse

class LoadUnloadOverlay(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)

    @ros_service(type = Trigger, name = 'load')
    def load(self):
        self.overlaid_object.relative_position = (100.0,10000.0,10.10);
        object_name = self.overlaid_object.grab()
        success = object_name != None
        return TriggerResponse(success, 'Grasped object: ' + str(object_name))

    @ros_service(type = Trigger, name = 'unload')
    def unload(self):

        success = self.overlaid_object.release()
        return TriggerResponse(success, '')

    @ros_service(type = Trigger, name = 'get_gripper_status')
    def get_status(self):
        if self.overlaid_object._grabbed_object:
            return TriggerResponse(True, self.overlaid_object._grabbed_object.name)
        else:
            return TriggerResponse(False, 'None')
