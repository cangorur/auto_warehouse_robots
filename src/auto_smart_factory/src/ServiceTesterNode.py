#!/usr/bin/env python3

"""This file creates a TesterNode which can be used to test services,
messages etc. It is good for unit testing of ROS nodes."""

import rospy
from auto_smart_factory.msg import RoadmapGraph
from auto_smart_factory.srv import *
from geometry_msgs.msg import Point, Twist, Vector3
from multiprocessing import Queue, Process
import random


def createPoint(x, y):
    """Utility function to create instances of geometry_msgs/Point."""
    p = Point()
    p.x = x
    p.y = y
    p.z = 0
    return p

def timeout(func):
    """This function is used for the decorator @timeout. With this decorator,
    waiting for a service call can be limited to the number of seconds which
    is specified below as time = ... ."""

    def call_with_timeout(*args, **kwargs):
        time = 10
        q = Queue()
        if not kwargs:
            kwargs = {"logger_queue": q}
        else:
            kwargs["logger_queue"] = q
        p = Process(target=func, args=args, kwargs=kwargs)
        p.start()
        p.join(time)

        if p.is_alive():
            p.terminate()
            rospy.logerr("Timeout reached " + str(time) + "s")

        for level, msg in q.get():
            if level == 0:
                rospy.loginfo(msg)
            elif level == 1:
                rospy.logwarn(msg)
            elif level == 2:
                rospy.logerr(msg)


    return call_with_timeout


@timeout
def call_service(service_name, service_class, args, logger_queue=None):
    """Call a service and wait for response.

    Args:
        service_name: str, the name of the service, not the type
        service_class: str, Python name of the service, i.e. the name of the .srv file
        args: tuple containing the arguments that shall be passed to the service
        logger_queue: multiprocessing.Queue

    Returns:
        If logger_queue is provided, return values of the service are put there.
    """

    logs = []
    logs.append((0, "Waiting for service " + service_name))
    rospy.loginfo("Waiting for service " + service_name)
    rospy.wait_for_service(service_name)
    logs.append((0, "Testing service " + service_name + " with arguments " + str(args)))
    rospy.loginfo("Testing service " + service_name + " with arguments " + str(args))
    try:
        func = rospy.ServiceProxy(service_name, service_class)
        result = func(*args)
        logs.append((0, "Success in calling " + service_name + ". Returned " + str(result)))
        rospy.loginfo("Success in calling " + service_name + ". Returned " + str(result))
    except rospy.ServiceException as e:
        rospy.logerr(str(e))
        logs.append((2, str(e)))
    if logger_queue:
        logger_queue.put(logs)


def createTwistMessage():
    t = Twist()
    lin = Vector3()
    ang = Vector3()
    lin.x = 0.0
    t.linear = lin
    t.angular = ang
    return t


def run_tests():

    #######################
    # Test services
    #######################
    rospy.loginfo("TesterNode started")

    r = rospy.Rate(0.1)
    r.sleep()
    r = rospy.Rate(4)

    for j in range(3):
        for i in range(1, 7):
            publisher = rospy.Publisher("robot_" + str(i) + "/motion", Twist, queue_size=1)
            publisher.publish(createTwistMessage())
            r.sleep()


    #r = rospy.Rate(3)
    rospy.loginfo("Start Testing")
    #for i in range(1, 1):
        #call_service("path_planner/request_new_path", RequestNewPath,
         #            args=("robot_" + str(i), False, createPoint(random.random()*16, random.random()*11.5), createPoint(random.random()*16, random.random()*11.5)))

    """for i in range(2, 5):
        call_service("robot_" + str(i) + "/perform_task_test", PerformTaskTest,
                     #args=(35 + i, createPoint(4, 5), createPoint(7, 7)))
                    args=(35 + i, createPoint(random.random()*8, random.random()*5), createPoint(7, 7)))   # task_id, start, end"""

    #call_service("robot_6/perform_task_test", PerformTaskTest,
     #            args=(1, createPoint(3, 4), createPoint(5, 3)))


    rospy.logwarn("After Testing")


    #call_service("traffic_planner/get_random_points", getRandomPoints,
     #              args=(rospy.get_rostime(), createPoint(random.random()*16, random.random()*11.5), createPoint(random.random()*16, random.random()*11.5), True))






    #call_service("test_service/cpp_test_service", RequestNewPath,
     #            args=("hallihallo", True, createPoint(1, 2), createPoint(3, 4)))

    # call_service("path_planner/init", InitPathPlanner, args=())    # should not be called because it interferes with warehouse management


    # GetCurrentETAStatus
    # call_service("robot_1/get_current_ETA_status", GetCurrentETAStatus,
    #              args=("robot_1",))

    # GetRandomPoints
    # call_service("traffic_planner/get_random_points", getRandomPoints,
    #              args=(rospy.get_rostime(), createPoint(1,2), createPoint(1,0), True))

    # CalculateETA
    # call_service("eta/calculate_ETA", CalculateETA,
    #              args=(createPoint(1, 2), createPoint(1, 0), ["robot_1", "robot_2"]))


    # getCurrentTraffic
    # GetTaskETA
    # RequestNewPath






if __name__ == "__main__":

    rospy.init_node("service_tester_node")
    rospy.loginfo("Started Service Tester Node")
    #run_tests()    # uncomment this line to enable tests
    rospy.spin()
