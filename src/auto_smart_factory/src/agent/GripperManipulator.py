#!/usr/bin/env python3

# Created by M.AL Dakhil

from auto_smart_factory.srv import *
from time import sleep
import rospy
import pymorse

mutex1 = 0


def handleMoveGripper(req):
    '''
	This service moves the gripper (changing their location in 3D simu env) by calling a morse service (set_object_position)
	Parameters:
		req: double x, double y, double z
	Returns:
		boolean success
    '''
    global mutex1
    #rospy.loginfo(req.gripper_id)
    while mutex1 == 1:
        sleep(0.01)
    mutex1 = 1
    success = True
    try:
        with pymorse.Morse() as sim:
            sim.rpc('simulation', 'set_object_position', req.gripper_id, [req.x, req.y, req.z])
    except:
        success = False
        
    if not success: 
        rospy.loginfo("[gripper manipulator]: MoveGripper {} success: {}!".format(req.gripper_id, success))
    
    mutex1 = 0
    return MoveGripperResponse(success)


def addMoveGripperServer():
    s = rospy.Service('~move_gripper', MoveGripper, handleMoveGripper)


if __name__ == "__main__":
    mutex1 = 0
    rospy.init_node('gripper_manipulator')
    addMoveGripperServer()

    #rospy.loginfo("GripperManipulator ready!")
    rospy.spin()
