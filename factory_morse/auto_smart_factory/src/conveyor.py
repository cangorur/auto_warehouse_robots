from morse.builder import *


class Conveyor(GroundRobot):
    """
    A template robot model for band, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # band.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'robots/conveyor.blend', name)
        self.properties(classpath ="morse.robots.conveyor_srv.Conveyor")
