import logging; logger = logging.getLogger("morse." + __name__)
from morse.builder import Robot

class RotatingTable(Robot):

    def __init__(self, name=None, debug = True):
        Robot.__init__(self, 'robots/rotating_table.blend', name)
        self.properties(classpath = "morse.robots.rotating_table_services.RotatingTable")
