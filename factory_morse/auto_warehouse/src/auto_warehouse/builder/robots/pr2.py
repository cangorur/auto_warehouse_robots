import logging; logger = logging.getLogger("morse." + __name__)
from morse.robots.grasping_robot import GraspingRobot
from morse.core.services import service
from morse.core import blenderapi

from morse.builder import *
from morse.builder.sensors import *
from morse.builder.actuators import *

import time
import math


class PR2(GraspingRobot):
    """ 
    The MORSE model of the Willow Garage's PR2 robot.

    The PR2 uses the :doc:`armature_actuator <../actuators/armature>`
    for control of the armatures.

    Model Info
    ----------

    The model is imported from a Collada file that is generated from the
    `PR2 URDF file  <http://www.ros.org/wiki/pr2_description>`_.
    The .dae file can be found at:
    ``$MORSE_ROOT/data/robots/pr2/pr2.dae``
    The imported .blend file can be found at:
    ``$MORSE_ROOT/data/robots/pr2/pr2_25_original.blend``

    The URDF to Collada converter changed all the object names, so these
    were remapped to the original URDF names. A renamed version of the
    PR2 model can be found at:
    ``$MORSE_ROOT/data/robots/pr2/pr2_25_rename.blend`` , this file
    includes the script that is used to rename all the objects.

    A model with MORSE integration for the armature can be found at
    (**This is the model that you probably want to use in MORSE**):
    ``$MORSE_ROOT/data/robots/pr2/pr2_25_morse.blend``.

    TODO
    ----

    - Create sensors and actuators to control the PR2 armature. `A
      SensorActuator class would be handy for this
      <https://sympa.laas.fr/sympa/arc/morse-users/2011-07/msg00099.html>`_.
    - Expand the armature to include the hands.
    - Add an actuator to control the movement of the PR2 base.
    - ROS integration.
    - ...

    """

    _name = 'PR2 robot'

    def __init__(self, obj, parent=None):
        """ 
        Constructor method.
        Receives the reference to the Blender object.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        GraspingRobot.__init__(self, obj, parent)

        """
        # We define here the name of the pr2 grasping hand:
        """
        self.hand_name = 'Hand.Grasp.PR2'

        self.armatures = []
        # Search armatures and torso in all objects parented to the pr2 empty
        for obj in self.bge_object.childrenRecursive:
            # Check if obj is an armature
            if type(obj).__name__ == 'BL_ArmatureObject':
                self.armatures.append(obj.name)
                logger.info(obj.name)
                logger.info(' ==\n')
            if obj.name == 'torso_lift_joint':
                self.torso = obj
                logger.info(' ==\n')

        # constant that holds the original height of the torso from the ground
        # These values come from the pr2 urdf file
        self.TORSO_BASE_HEIGHT = (0.739675 + 0.051)
        self.TORSO_LOWER = 0.0  # lower limit on the torso z-translantion
        self.TORSO_UPPER = 0.31  # upper limit on the torso z-translation
        
        self.MIN_DIST = 0.85
        self.cancel = False
        self.is_ho = False
        self.is_gr = False # is grasping?

        # raise both hands up
        scene = blenderapi.scene()

        hand_l = scene.objects['l_elbow_flex_joint']
        hand_l.worldOrientation = [math.radians(0), math.radians(-90), math.radians(180)]

        hand_r = scene.objects['r_elbow_flex_joint']
        hand_r.worldOrientation = [math.radians(90), math.radians(-90), math.radians(-90)]
        
        head = scene.objects['head_tilt_joint']
        head.worldOrientation = [0, 0, math.radians(90)]

        logger.info('Component initialized')

    @service
    def cancel_action(self):
        self.cancel = True
        #self.obj_reset()
        time.sleep(2)
        self.reset()

    @service
    def reset(self):
        """ reset the robot. """

        scene = blenderapi.scene()

        hand_l = scene.objects['l_elbow_flex_joint']
        hand_l.worldOrientation = [math.radians(0), math.radians(-90), math.radians(180)]

        hand_r = scene.objects['r_elbow_flex_joint']
        hand_r.worldOrientation = [math.radians(90), math.radians(-90), math.radians(-90)]

        shoulder_l = scene.objects['l_shoulder_pan_joint']
        upper_arm_l = scene.objects['l_upper_arm_roll.001']
        upper_arm_r = scene.objects['r_upper_arm_roll.001']
        shoulder_r = scene.objects['r_shoulder_pan_joint']
        pr2 = scene.objects['robot']

        shoulder_l.worldOrientation = [math.radians(0), math.radians(0), math.radians(90)]  # at 0 degrees (idle)
        upper_arm_l.worldOrientation = [math.radians(0), math.radians(0), math.radians(90)]
        shoulder_r.worldOrientation = [math.radians(0), math.radians(0), math.radians(90)]
        upper_arm_r.worldOrientation = [math.radians(0), math.radians(0), math.radians(90)]
        pr2.worldOrientation = [0, 0, math.radians(90)]

        head = scene.objects['head_tilt_joint']
        head.worldOrientation = [0, 0, math.radians(90)]

        self.cancel = False
        self.is_ho = False

    @service
    def obj_reset(self):
        ''' ungrasp object '''
        scene = blenderapi.scene()
        obj = scene.objects['package1']
        obj.removeParent()
        obj.worldPosition = [7.7, -2.1, 0.80]

    @service
    def distance(self):
        ''' Distance between object and hand '''

        scene = blenderapi.scene()
        hand_l_fr = scene.objects['l_gripper_tool_fr.000']

        obj = scene.objects['package1']

        vec1 = hand_l_fr.worldPosition
        vec2 = obj.worldPosition

        dist = vec1 - vec2
        dist_value = math.sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2])

        return dist_value

    def finger_open(self):

        scene = blenderapi.scene()
        finger_l = scene.objects['l_grip_l_fing_joint']
        finger_r = scene.objects['l_grip_r_fing_joint']
        finger_rl = scene.objects['r_grip_l_fing_joint']
        finger_rr = scene.objects['r_grip_r_fing_joint']

        N = 10
        # open finger
        for i in range(N):

            if self.cancel:
                self.reset()
                return

            finger_l.applyRotation( [0, 0, 0.5 / N], True )
            finger_r.applyRotation( [0, 0, -0.5 / N], True )
            finger_rl.applyRotation([0, 0, 0.5 / N], True)
            finger_rr.applyRotation([0, 0, -0.5 / N], True)
                
            time.sleep(0.1)

    def finger_close(self):

        scene = blenderapi.scene()
        finger_l = scene.objects['l_grip_l_fing_joint']
        finger_r = scene.objects['l_grip_r_fing_joint']
        finger_rl = scene.objects['r_grip_l_fing_joint']
        finger_rr = scene.objects['r_grip_r_fing_joint']

        N = 10
        # close finger
        for i in range(N):

            if self.cancel:
                self.reset()
                return

            finger_l.applyRotation( [0, 0, -0.5 / N], True )
            finger_r.applyRotation( [0, 0, 0.5 / N], True )
            finger_rl.applyRotation([0, 0, -0.5 / N], True)
            finger_rr.applyRotation([0, 0, 0.5 / N], True)
                
            time.sleep(0.1)

    @service
    def grasp(self):
        ''' grasp object '''

        scene = blenderapi.scene()
        hand_l = scene.objects['l_elbow_flex_joint']
        hand_l_fr = scene.objects['l_gripper_tool_fr.000']
        hand_r = scene.objects['r_elbow_flex_joint']
        hand_r_fr = scene.objects['r_gripper_tool_fr.000']
        head = scene.objects['head_tilt_joint']
        head.worldOrientation = [0, math.radians(10), math.radians(90)]
        shoulder_l = scene.objects['l_shoulder_pan_joint']
        upper_arm_l = scene.objects['l_upper_arm_roll.001']
        upper_arm_r = scene.objects['r_upper_arm_roll.001']
        shoulder_r = scene.objects['r_shoulder_pan_joint']

        wrist_l = scene.objects['l_wrist_roll_joint']

        pr2 = scene.objects['robot']

        obj = scene.objects['package1']

        N = 20

        # approach to the object with both arms
        for i in range(N):

            if self.cancel:
                self.reset()
                self.reset()
                return False
            hand_l.applyRotation([0, 0, -1.55 / N], True)
            hand_r.applyRotation([0, 0, 1.55 / N], True)
            shoulder_l.applyRotation([0, 0, math.radians(20) / N], True) # at 20 degrees (open)
            upper_arm_l.applyRotation([0, 0, math.radians(20) / N], True)
            shoulder_r.applyRotation([0, 0, math.radians(-20) / N], True)
            upper_arm_r.applyRotation([0, 0, math.radians(-20) / N], True)

            time.sleep(0.06)

        # back
        dist = self.distance()

        if (dist < self.MIN_DIST):

            # close the shoulders to hold the object
            N = 20
            for i in range(N):
                if self.cancel:
                    self.reset()
                    self.reset()
                    self.reset()
                    return True
                shoulder_l.applyRotation([0, 0, math.radians(-20) / N], True) # at 0 degrees (idle)
                upper_arm_l.applyRotation([0, 0, math.radians(-20) / N], True)
                shoulder_r.applyRotation([0, 0, math.radians(20) / N], True)
                upper_arm_r.applyRotation([0, 0, math.radians(20) / N], True)
                time.sleep(0.1)
            # self.finger_open()

            obj.setParent(wrist_l)
            obj.worldPosition[0] = wrist_l.worldPosition[0] + 0.25
            obj.worldPosition[1] = wrist_l.worldPosition[1] + 0.1
            N = 20
            # lift the object up
            self.is_ho = True
            for i in range(N):
                if self.cancel:
                    self.reset()
                    self.obj_reset()
                    self.reset()
                    self.reset()
                    return True
                hand_l.applyRotation([0, 0, math.radians(45) / N], True)
                hand_r.applyRotation([0, 0, math.radians(-45) / N], True)
                # obj.worldPosition = [hand_l.worldPosition[0] + 0.05, hand_l.worldPosition[1],
                #                     hand_l.worldPosition[2]]
                time.sleep(0.1)

            time.sleep(0.1)
            # Turning towards the processed storage tray
            for i in range(N):
                # When turning the robot cannot see human warning
                if self.cancel:
                    self.reset()
                    self.obj_reset()
                    self.reset()
                    self.reset()
                    return True
                pr2.applyRotation([0, 0, math.radians(90) / N], True)
                time.sleep(0.1)

            # dropping the object off
            for i in range(N):
                # When turned the robot cannot see the human warning
                if self.cancel:
                    self.reset()
                    self.reset()
                    self.reset()
                    return True
                shoulder_l.applyRotation([0, 0, math.radians(20) / N], True) # at 20 degrees (open)
                upper_arm_l.applyRotation([0, 0, math.radians(20) / N], True)
                shoulder_r.applyRotation([0, 0, math.radians(-20) / N], True)
                upper_arm_r.applyRotation([0, 0, math.radians(-20) / N], True)
                if i == 2:
                    obj.removeParent()
                time.sleep(0.1)
            time.sleep(0.5)
            # obj.worldPosition = [7.7 - 0.6, -3.04, 0.80]

            # Turning back to the conveyor belt
            for i in range(N):
                if self.cancel:
                    self.reset()
                    self.reset()
                    self.reset()
                    return True
                pr2.applyRotation([0, 0, math.radians(-90) / N], True)
                hand_l.applyRotation([0, 0, math.radians(45) / N], True)
                hand_r.applyRotation([0, 0, math.radians(-45) / N], True)
                shoulder_l.applyRotation([0, 0, math.radians(-20) / N], True) # at 0 degrees (idle)
                upper_arm_l.applyRotation([0, 0, math.radians(-20) / N], True)
                shoulder_r.applyRotation([0, 0, math.radians(20) / N], True)
                upper_arm_r.applyRotation([0, 0, math.radians(20) / N], True)
                time.sleep(0.1)

            #self.finger_close()
            head.worldOrientation = [0, 0, math.radians(90)]
            self.is_ho = False
            return True
        else:
            self.reset()
            self.reset()
            self.is_ho = False
            return False

    @service
    def pointToObj(self):
        ''' point hand to object '''

        scene = blenderapi.scene()
        hand_l = scene.objects['l_elbow_flex_joint']
        hand_l_fr = scene.objects['l_gripper_tool_fr.000']

        obj = scene.objects['package1']

        N = 5
        # fetch
        for i in range(N):
            if self.cancel:
                self.reset()
                return False
            hand_l.applyRotation([0, 0, math.radians(-90) / N], True)
            time.sleep(0.1)

        time.sleep(0.5)
        for i in range(N):
            if self.cancel:
                self.reset()
                return True
            hand_l.applyRotation([0, 0, math.radians(90) / N], True)
        time.sleep(0.5)
        for i in range(N):
            if self.cancel:
                self.reset()
                return False
            hand_l.applyRotation([0, 0, math.radians(-90) / N], True)
            time.sleep(0.1)

        time.sleep(0.5)
        for i in range(N):
            if self.cancel:
                self.reset()
                return True
            hand_l.applyRotation([0, 0, math.radians(90) / N], True)
        time.sleep(0.5)

    @service
    def get_armatures(self):
        """
        Returns a list of all the armatures on the PR2 robot.
        """
        return self.armatures

    @service
    def set_torso(self, height):
        """
        MORSE Service that sets the z-translation of the torso to original_z + height.
        """
        if self.TORSO_LOWER < height < self.TORSO_UPPER:
            self.torso.localPosition = [-0.05, 0, self.TORSO_BASE_HEIGHT + height]
            return "New torso z position: " + str(self.torso.localPosition[2])
        else:
            return "Not a valid height, value has to be between 0.0 and 0.31!"
            
    @service
    def get_torso(self):
        """
        Returns the z-translation of the torso.
        """
        return self.torso.localPosition[2] - self.TORSO_BASE_HEIGHT

    @service
    def get_torso_minmax(self):
        """
        Returns the minimum an maximum z-translation that the torso can
        make from the base.  Returns a list [min,max]
        """
        return [self.TORSO_LOWER, self.TORSO_UPPER]

    def default_action(self):
        """
        Main function of this component.
        """
        pass
