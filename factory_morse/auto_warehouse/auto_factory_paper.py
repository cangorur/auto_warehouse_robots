#! /usr/bin/env morseexec

from morse.builder import *
from map_setup import setup_warehouse_map
from package_setup import add_packages
import json

# specify configuration files
warehouse_config_file = os.path.join(os.path.dirname(__file__), '../../configs/smart_factory_config_paper.json')
robot_config_file = os.path.join(os.path.dirname(__file__), '../../configs/robot_config.json')
package_config_file = os.path.join(os.path.dirname(__file__), '../../configs/package_config.json')

# read configurations from JSON files
with open(warehouse_config_file, 'r') as f:
    map_config = json.load(f)
with open(robot_config_file, 'r') as f:
    robot_config = json.load(f)
with open(package_config_file, 'r') as f:
    package_configs = json.load(f)

simplfy_flag = True
# setup map
setup_warehouse_map(map_config, robot_config, simplfy_flag)
add_packages(map_config, package_configs)

# set 'fastmode' to True to switch to wireframe mode
env = Environment('empty_environment.blend', fastmode = False)

map_size_x = map_config['map']['width']
map_size_y = map_config['map']['height']

env.set_camera_location([-2., 8.0, 4.5])
env.set_camera_rotation([0.95, 0, -1.2])
 
