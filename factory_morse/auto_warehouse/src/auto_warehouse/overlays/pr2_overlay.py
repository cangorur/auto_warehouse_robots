from morse.middleware.ros_request_manager import ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

from std_srvs.srv import Trigger, TriggerResponse


class RobotControlAndMonitor(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)

        # if is object at hand
        self.is_obj = False

        # if current action is executed
        self.is_gr = False  # has grasped
        self.is_po = False  # has pointed out
        self.is_ca = False  # has canceled

    @ros_service(type=Trigger, name='cancel_action')
    def cancel_action(self):
        if not self.is_ca:
            self.is_ca = True
            self.overlaid_object.cancel_action()

            self.is_obj = False
            self.is_gr = False
            self.is_po = False
            self.is_ca = False

            return TriggerResponse(True, 'robot: cancel action')
        else:
            return TriggerResponse(True, 'robot: action(cancel action) is not finished')

    @ros_service(type=Trigger, name='is_ho')
    def is_ho(self):
        if self.overlaid_object.is_ho:
            return TriggerResponse(True, 'robot: is_ho True')
        else:
            return TriggerResponse(False, 'robot: is_ho False')

    @ros_service(type=Trigger, name='reset')
    def reset(self):
        self.overlaid_object.reset()
        self.is_obj = False
        self.is_gr = False
        self.is_po = False
        self.is_ca = False
        return TriggerResponse(True, 'robot: reset')

    # distance
    @ros_service(type=Trigger, name='distance')
    def distance(self):
        dist = self.overlaid_object.distance()
        if (dist < 1.2):
            return TriggerResponse(True, 'robot: object in range' + str(dist))
        else:
            return TriggerResponse(True, 'robot: object out of range' + str(dist))

    # grasp
    @ros_service(type=Trigger, name='grasp')
    def grasp(self):
        if self.is_obj:
            return TriggerResponse(True, 'robot: already had object')
        else:
            if not self.is_gr:
                self.is_gr = True
                success = self.overlaid_object.grasp()
                self.is_gr = False

                if success:
                    self.is_obj = True
                    return TriggerResponse(True, 'robot: grasp object')
                else:
                    return TriggerResponse(True, 'robot: object out of range')
            else:
                return TriggerResponse(True, 'robot: action(grasp object) is not finished')

    # point to object
    @ros_service(type=Trigger, name='point_to_obj')
    def point_to_obj(self):
        if not self.is_po:
            self.is_po = True
            self.overlaid_object.pointToObj()
            self.is_po = False
            return TriggerResponse(True, 'robot: point to object')
        else:
            return TriggerResponse(True, 'robot: action(point to object) is not finished')
