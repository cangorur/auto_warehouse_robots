import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

class Custombattery(morse.core.sensor.Sensor):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "Custom battery sensor"
    _short_desc = "Custom battery sensor with separately configurable charging rate."
    
    add_property('_discharging_rate', 0.01, 'DischargingRate', "float", "Battery discharging rate, in percent per seconds")
    add_property('_charging_rate', 3.0, 'ChargingRate', "float", "Battery charging rate, in percent per seconds")
    add_property('_distance_discharging_factor', 0.1, 'MotorDrainingRate', "float", "Battery discharging factor related to distance covered by robots")
    
    add_data('charge', 100.0, "float", "Initial battery level, in percent")
    add_data('status', "Charged", "string", "Charging Status")
    add_data('last_pose_x', 0.0, "float", "Last position x of the robot")
    add_data('last_pose_y', 0.0, "float", "Last position y of the robot")


    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self._time = self.robot_parent.gettime()
        # Initialization of the previous pose 
        self.local_data['last_pose_x'] = self.robot_parent.position_3d.x
        self.local_data['last_pose_y'] = self.robot_parent.position_3d.y

        logger.info('Component initialized, runs at %.2f Hz, discharging rate = %f, motor drain rate = %f', self.frequency, self._discharging_rate, self._distance_discharging_factor)

    def default_action(self):
        """ Main function of this component. """
        charge = self.local_data['charge']
        dt = self.robot_parent.gettime() - self._time
        pose_x = self.robot_parent.position_3d.x - self.local_data['last_pose_x']
        pose_y = self.robot_parent.position_3d.y - self.local_data['last_pose_y']
        pose = math.sqrt(pose_x* pose_x + pose_y* pose_y)
        # logger.info('previous pose = [%f, %f] ,, current pose =[%f, %f]', self.local_data['last_pose_x'], self.local_data['last_pose_y'], self.robot_parent.position_3d.x, self.robot_parent.position_3d.y)
        # Set the current location as previous for the next step
        self.local_data['last_pose_x'] = self.robot_parent.position_3d.x
        self.local_data['last_pose_y'] = self.robot_parent.position_3d.y

        if self.in_zones(type = 'Charging'):
            charge = charge + dt * self._charging_rate
            status = "Charging"
            if charge > 100.0:
                charge = 100.0
                status = "Charged"
        else:
            charge = charge - pose * self._distance_discharging_factor - dt * self._discharging_rate
            status = "Discharging"
            if charge < 0.0:
                charge = 0.0

        # Store the data acquired by this sensor that could be sent
        #  via a middleware.
        self.local_data['charge'] = float(charge)
        self.local_data['status'] = status
        # update the current time
        self._time = self.robot_parent.gettime()

