from morse.builder import *


class ConveyorDistributor(GroundRobot):
    """
    A template robot model for band, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # band.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'auto_warehouse/robots/conveyor_distributor.blend', name)
        self.properties(classpath ="auto_warehouse.robots.conveyor_srv.Conveyor")
