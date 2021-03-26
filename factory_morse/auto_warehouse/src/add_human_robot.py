import math
from morse.builder import *
from auto_warehouse.builder.robots.human import *
from auto_warehouse.builder.robots.pr2 import *

def create_human_and_robot(map_config):
    # Creating a human model
    i = 0
    for human_config in map_config['human_robot_collaboration']['humans']:
        conveyor = map_config['human_robot_collaboration']['conveyors'][i]
        #human = Human(name = human_config['id'])
        human = Human()
        if i == 0:
            human_x = conveyor['location']['x'] - 0.7
            human_y = conveyor['location']['y'] - 1.5 # -0.5
        else:
            human_x = conveyor['location']['x'] + 0.7
            human_y = conveyor['location']['y'] - 1.5
        human.translate(human_x, human_y, 0)
        human.rotate(0, 0, math.radians(human_config['orientation']))
        human.add_overlay('ros', 'auto_warehouse.overlays.human_overlay.HumanControlAndMonitor')
        human.disable_keyboard_control()
        # human.use_world_camera()
        # Default interface
        # human.add_default_interface('ros')
        # human.add_service('socket')
        # human.add_service('ros')
        i += 1

    i = 0
    for robot_config in map_config['human_robot_collaboration']['pr2_robots']:
        conveyor = map_config['human_robot_collaboration']['conveyors'][i]
        # Creating a PR2 robot model
        robot = BasePR2(robot_config['id'])
        robot_x = conveyor['location']['x'] + 1
        robot_y = conveyor['location']['y'] - 1.2 # -2.3
        robot.translate(robot_x, robot_y, 0.05)
        robot.rotate(0, 0, math.radians(robot_config['orientation']))
        robot.add_overlay('ros', 'auto_warehouse.overlays.pr2_overlay.RobotControlAndMonitor')
        # Default interface
        robot.add_default_interface('ros')
        # Service
        robot.add_service('ros')
        i += 1
