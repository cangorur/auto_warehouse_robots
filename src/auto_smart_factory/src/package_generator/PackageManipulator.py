#!/usr/bin/env python3

from auto_smart_factory.srv import *
import rospy
import pymorse

mutex = 0


def handleMovePackage(req):
    '''
	This service moves the packages (changing their location in 3D simu env) by calling a morse service (set_object_position)
	Parameters:
		req: double x, double y, double z
	Returns:
		boolean success
    '''
    global mutex
    rospy.loginfo(req.package_id)
    while mutex == 1:
        pass
        #rospy.loginfo("waiting")
    mutex = 1
    success = True
    try:
        with pymorse.Morse() as sim:
            sim.rpc('simulation', 'set_object_position', req.package_id, [req.x, req.y, req.z])
    except:
        success = False
    rospy.logdebug("[package manipulator]: MovePackage {} success: {}!".format(req.package_id, success))
    mutex = 0
    return MovePackageResponse(success)


def addMovePackageServer():
    s = rospy.Service('~move_package', MovePackage, handleMovePackage)


if __name__ == "__main__":
    rospy.init_node('package_manipulator')
    addMovePackageServer()
    rospy.loginfo("PackageManipulator ready!")
    rospy.spin()
