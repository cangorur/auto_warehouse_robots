import logging

logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.robots.grasping_robot import GraspingRobot
from morse.core.services import service

import time
import math

PI = math.pi


class Human(GraspingRobot):
    """ Class definition for the human as a robot entity.

    Sub class of GraspingRobot.
    """

    def __init__(self, obj, parent=None):
        """ Call the constructor of the parent class """
        logger.info('%s initialization' % obj.name)
        GraspingRobot.__init__(self, obj, parent)

        # We define here the name of the human grasping hand:
        self.hand_name = 'Hand_Grab.R'

        self.MIN_DIST = 0.8

        # these are to feed the human action back to the system (current state)
        self.is_wa = False  # is human walking away
        self.is_gr = False  # is human grasping

        self.is_la = False  # is human looking around
        self.is_wr = False  # is human warning the robot
        self.is_ag = False  # is human attempting to grasp

        self.is_ov = True  # is the object visible to human
        self.is_oir = True  # is the object reachable to human
        self.is_ho = False  # if human has the object

        logger.info('Component initialized')

    @service
    def reset(self):
        ''' ungrasp object '''

        if self.is_la:
            self.look_back()

        if self.is_ag:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        if self.is_wa:
            self.walk_back()

        self.is_wa = False
        self.is_gr = False

        self.is_la = False
        self.is_wr = False
        self.is_ag = False

        self.is_ov = True
        self.is_oir = True
        self.is_ho = False

        human = self.bge_object
        human.worldPosition = [7.7, -1.25, 0]
        human.worldOrientation = [0, 0, -1.57]

    @service
    def obj_reset(self):
        ''' ungrasp object '''
        scene = blenderapi.scene()
        obj = scene.objects['package1']
        obj.worldPosition = [7.7, -2.1, 0.80]

    @service
    def stay_idle(self):
        """ Move human to left. """

        if self.is_la:
            self.look_back()

        if self.is_ag:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        self.is_ov = True
        #self.is_oir = True

    @service
    def walk_away(self):
        """ Move human to left. """

        if self.is_wa:
            return
        else:
            self.is_wa = True

        if self.is_la:
            self.look_back()

        if self.is_ag:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        self.is_ov = False
        self.is_oir = False

        human = self.bge_object

        N = 10
        # turn left 180 degree
        for i in range(10):
            human.applyRotation([0, 0, math.radians(180) / N], True)
            time.sleep(0.1)

        time.sleep(0.3)

        N = 20
        # go ahead 3.5m
        for i in range(N):
            human.applyMovement([3.5 / N, 0, 0], True)
            time.sleep(0.1)

        time.sleep(0.3)

        N = 10
        # turn right 90 degree
        for i in range(N):
            human.applyRotation([0, 0, math.radians(90) / N], True)
            time.sleep(0.1)

        time.sleep(0.3)

        N = 20
        # go ahead 2m
        for i in range(N):
            human.applyMovement([2 / N, 0, 0], True)
            time.sleep(0.1)

    @service
    def walk_back(self):
        """ Move human to left. """

        if self.is_wa:
            self.is_wa = False
        else:
            return
        human = self.bge_object

        N = 10
        # turn 180 degree
        for i in range(10):
            human.applyRotation([0, 0, math.radians(180) / N], True)

            time.sleep(0.1)

        time.sleep(0.3)

        N = 20
        # go ahead 2m
        for i in range(N):
            human.applyMovement([2 / N, 0, 0], True)

            time.sleep(0.1)

        N = 10
        # turn right 90 degree
        for i in range(N):
            human.applyRotation([0, 0, math.radians(-90) / N], True)

            time.sleep(0.1)

        time.sleep(0.3)

        N = 20
        # go ahead 3.5m
        for i in range(N):
            human.applyMovement([3.5 / N, 0, 0], True)

            time.sleep(0.1)

        time.sleep(0.3)
        self.is_ov = True
        self.is_oir = True

    @service
    def move(self, speed, rotation):
        """ Move the human. """

        human = self.bge_object

        if not human['Manipulate']:
            human.applyMovement([speed, 0, 0], True)
            human.applyRotation([0, 0, rotation], True)
        else:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']

            target.applyMovement([0.0, rotation, 0.0], True)
            target.applyMovement([0.0, 0.0, -speed], True)

    @service
    def look_around(self):
        """ Move the human head to look around. """

        if self.is_la:
            return

        if self.is_ag:
            self.attempt_grasp_back()

        if self.is_wr:
            self.warn_robot_back()

        scene = blenderapi.scene()
        target = scene.objects['Look_Empty']

        # look left
        N = 10
        for i in range(N):
            target.applyMovement([0, PI / 2 / N, 0], True)
            time.sleep(0.1)

        self.is_la = True
        self.is_ov = False

    def look_back(self):
        """ Move the human head to look back. """

        scene = blenderapi.scene()
        target = scene.objects['Look_Empty']

        # look right
        N = 10
        for i in range(N):
            target.applyMovement([0, -1 * PI / 2 / N, 0], True)
            time.sleep(0.1)

        self.is_la = False
        self.is_ov = True

    @service
    def move_head(self, pan, tilt):
        """ Move the human head. """

        human = self.bge_object
        scene = blenderapi.scene()
        target = scene.objects['Target_Empty']

        if human['Manipulate']:
            return

        target.applyMovement([0.0, pan, 0.0], True)
        target.applyMovement([0.0, 0.0, tilt], True)

    def distance(self):
        ''' Distance between object and hand '''

        scene = blenderapi.scene()
        hand = scene.objects['IK_Target_Empty.R']
        obj = scene.objects['package1']

        vec1 = hand.worldPosition
        vec2 = obj.worldPosition

        dist = vec1 - vec2
        dist_value = math.sqrt(dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2])

        return dist_value

    @service
    def warn_robot(self):
        ''' corss both hands to warn robot'''

        if self.is_wr:
            return
        else:
            self.is_wr = True

        if self.is_la:
            self.look_back();

        if self.is_ag:
            self.attempt_grasp_back()

        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        dest = scene.objects['IK_Pose_Empty.R']
        target = scene.objects['Target_Empty']

        obj = scene.objects['package1']

        ''' fetch hand '''
        vec1 = hand_r.localPosition
        # vec2 = dest.localPosition
        vec2 = obj.localPosition

        f_speed = (vec2 - vec1)
        f_speed = [0.6, 0, 0.53]
        # fetch
        N = 10
        for i in range(N):
            target.applyMovement([0, 0, math.radians(90) / N], True)
            time.sleep(0.1)

        N = 10
        for i in range(N):
            hand_r.applyMovement([f_speed[0] / N, f_speed[1] / N, f_speed[2] / N], True)
            hand_l.applyMovement([f_speed[0] / N, f_speed[1] / N, f_speed[2] / N], True)

            time.sleep(0.1)

        time.sleep(1)
        self.is_wr = True

    @service
    def warn_robot_back(self):
        scene = blenderapi.scene()
        hand_l = scene.objects['IK_Target_Empty.L']
        hand_r = scene.objects['IK_Target_Empty.R']
        target = scene.objects['Target_Empty']

        N = 5
        for i in range(N):
            target.applyMovement([0, 0, math.radians(-90) / N], True)
            time.sleep(0.1)

        # fetch hand back
        hand_l.localPosition = [0, 0.2, 0.82]
        hand_r.localPosition = [0, -0.2, 0.82]

        self.is_wr = False

    @service
    def grasp(self):
        ''' grasp object '''
        self.is_gr = True

        if self.is_la:
            self.look_back()

        if self.is_wr:
            self.warn_robot_back()

        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        dest = scene.objects['IK_Pose_Empty.R']
        target = scene.objects['Target_Empty'] # head

        obj = scene.objects['package1']

        ''' fetch hand '''
        vec1 = hand_r.localPosition
        # vec2 = dest.localPosition
        vec2 = obj.localPosition

        f_speed = (vec2 - vec1)
        f_speed = [0.6, 0, 0.1]
        # fetch
        N = 5
        for i in range(N):
            target.applyMovement([0, 0, PI / 2 / N], True)
            time.sleep(0.1)

        N = 5
        for i in range(N):
            hand_r.applyMovement([f_speed[0] / N, f_speed[1] / N, f_speed[2] / N], True)
            hand_l.applyMovement([f_speed[0] / N, f_speed[1] / N, -f_speed[2] / N], True)

            time.sleep(0.1)

        time.sleep(1)

        self.is_gr = True

        # back
        dist = self.distance()

        # TODO: below the distance calculation will be fixed
        if dist < self.MIN_DIST:
            f_speed = [-0.4, 0, -0.1]
            obj.setParent(hand_r)
            N = 10
            for i in range(N):
                hand_r.applyMovement([f_speed[0] / N, f_speed[1] / N, f_speed[2] / N], True)
                hand_l.applyMovement([f_speed[0] / N, f_speed[1] / N, -f_speed[2] / N], True)
                obj.worldPosition = [hand_r.worldPosition[0] + 0.3, hand_r.worldPosition[1] - 0.2, hand_r.worldPosition[2] + 0.3]

                #if N == 5:
                #    hand_r.localPosition = [0.25, -0.2, 0.8]
                #    hand_l.localPosition = [0.25, 0.2, 0.8]
                time.sleep(0.1)

            #obj.worldPosition[1] = obj.worldPosition[1] - 0.1
            #time.sleep(1)

            self.is_ho = True
            # putting the object into the container
            human = self.bge_object
            hand_r.localPosition = [0.25, -0.2, 0.8]
            hand_l.localPosition = [0.25, 0.2, 0.8]
            obj.worldPosition = [hand_r.worldPosition[0] - 0.2, hand_r.worldPosition[1] - 0.3, hand_r.worldPosition[2] + 0.3]
            N = 5
            # turn right 90 degree
            for i in range(N):
                human.applyRotation([0, 0, math.radians(-90) / N], True)
                obj.worldPosition = [hand_r.worldPosition[0] - 0.3, hand_r.worldPosition[1] - 0.3, hand_r.worldPosition[2] + 0.3]
                time.sleep(0.1)

            obj.worldPosition[2] -= 0.1
            obj.removeParent()
            hand_r.localPosition = [0, -0.2, 0.82]
            hand_l.localPosition = [0, 0.2, 0.82]
            time.sleep(0.5)
            self.is_ho = False

            N = 5
            # turn left 90 degree
            for i in range(N):
                human.applyRotation([0, 0, math.radians(90) / N], True)
                time.sleep(0.1)

            human.worldPosition = [7.7, -1.25, 0]
            human.worldOrientation = [0, 0, -1.57]
            self.is_gr = False
            return True

        else:
            hand_r.localPosition = [0, -0.2, 0.82]
            hand_l.localPosition = [0, 0.2, 0.82]
            self.is_gr = False
            self.is_ho = False
            return False

    @service
    def attempt_grasp(self):
        ''' attempt to grasp object '''
        self.is_ag = True
        self.is_gr = True
        self.is_ho = False

        if self.is_la:
            self.look_back()

        if self.is_wr:
            self.warn_robot_back()

        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        dest = scene.objects['IK_Pose_Empty.R']
        target = scene.objects['Target_Empty']

        obj = scene.objects['package1']

        ''' fetch hand '''
        vec1 = hand_r.localPosition
        # vec2 = dest.localPosition
        vec2 = obj.localPosition

        f_speed = (vec2 - vec1)
        f_speed = [0.6, 0, 0.1]
        # fetch

        N = 5
        for i in range(N):
            target.applyMovement([0, 0, PI / 2 / N], True)
            time.sleep(0.1)

        N = 5
        for i in range(N):
            hand_r.applyMovement([f_speed[0] / N, f_speed[1] / N, f_speed[2] / N], True)
            hand_l.applyMovement([f_speed[0] / N, f_speed[1] / N, -f_speed[2] / N], True)
            time.sleep(0.1)

        if True: #(dist < self.MIN_DIST):

            f_speed = [-0.6, 0, -0.1]
            obj.setParent(hand_r)
            N = 10
            for i in range(N):
                hand_r.applyMovement([f_speed[0] / N, f_speed[1] / N, f_speed[2] / N], True)
                hand_l.applyMovement([f_speed[0] / N, f_speed[1] / N, -f_speed[2] / N], True)
                if i < 2:
                    obj.worldPosition = [hand_r.worldPosition[0] + 0.3, hand_r.worldPosition[1] - 0.2,
                                         hand_r.worldPosition[2] + 0.3]
                    time.sleep(1.5)
                else:
                    obj.removeParent()
                    time.sleep(0.1)

            obj.worldPosition = [7.7, -2.1, 0.80]
            hand_r.localPosition = [0, -0.2, 0.82]
            hand_l.localPosition = [0, 0.2, 0.82]

            time.sleep(0.5)
            self.is_ho = False
            self.is_ag = False
            self.is_gr = False
            return True

        else:
            hand_r.localPosition = [0, -0.2, 0.82]
            hand_l.localPosition = [0, 0.2, 0.82]
            self.is_gr = False
            self.is_ag = False
            self.is_ho = False
            return False


    @service
    def attempt_grasp_back(self):
        scene = blenderapi.scene()
        hand_r = scene.objects['IK_Target_Empty.R']
        hand_l = scene.objects['IK_Target_Empty.L']
        # back
        hand_r.localPosition = [0, -0.2, 0.82]
        hand_l.localPosition = [0, 0.2, 0.82]

        self.is_ag = False
        self.is_gr = False

    @service
    def move_hand(self, diff, tilt):
        """ Move the human hand (wheel).

        A request to use by a socket.
        Done for wiimote remote control.
        """

        human = self.bge_object
        if human['Manipulate']:
            scene = blenderapi.scene()
            target = scene.objects['IK_Target_Empty.R']
            target.applyMovement([diff, 0.0, 0.0], True)

    @service
    def toggle_manipulation(self):
        """ Change from and to manipulation mode.

        A request to use by a socket.
        Done for wiimote remote control.
        """

        human = self.bge_object
        scene = blenderapi.scene()
        hand_target = scene.objects['IK_Target_Empty.R']
        head_target = scene.objects['Target_Empty']

        # '''
        if human['Manipulate']:
            human['Manipulate'] = False
            # Place the hand beside the body
            hand_target.localPosition = [0.0, -0.3, 0.8]
            head_target.setParent(human)
            head_target.localPosition = [1.3, 0.0, 1.7]
        else:
            human['Manipulate'] = True
            head_target.setParent(hand_target)
            # Place the hand in a nice position
            hand_target.localPosition = [0.6, 0.0, 1.4]
            # Place the head in the same place
            head_target.localPosition = [0.0, 0.0, 0.0]
        # '''

        return human['Manipulate']

    @service
    def manipulation_state(self):
        """return state of human['Manipulate'] """
        human = self.bge_object

        return human['Manipulate']

    def default_action(self):
        """ Main function of this component. """
        pass
