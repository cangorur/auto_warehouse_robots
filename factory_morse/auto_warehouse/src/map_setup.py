import math
from morse.builder import *
from auto_warehouse.builder.robots import pioneer_robot, pioneer_light_robot, kuka_jido
import add_human_robot
from auto_warehouse.builder.robots.conveyor import *
#from morse.builder.robots.rotating_table import *
from auto_warehouse.builder.robots.conveyor_distributor import *

# width is size in x direction, height is size in y direction

def create_floor(width, height):
    segment_size_x = 1.
    segment_size_y = 1.

    size_x = math.ceil(width / segment_size_x)
    size_y = math.ceil(height / segment_size_y)

    for x in range(size_x):
        for y in range(size_y):
            floor_segment = PassiveObject('environment_segments/floor_segment.blend', 'floor')
            floor_segment.translate(x * segment_size_x, y * segment_size_y, 0.0)

# create the walls function
def create_walls(wall,width, height):
    # wall segmenting
    segment_size_x = 1.
    segment_size_y = 1.

    size_x = math.ceil(width / segment_size_x)
    size_y = math.ceil(height / segment_size_y)

    segment_size_x = 2.
    segment_size_y = 2.

# creating the fence of the warehouse environment
    for x in range(int(size_x / segment_size_x)):
        wall_segment = PassiveObject('environment_segments/wall_segment.blend', 'wall')
        wall_segment.translate(x * segment_size_x, 0.0, 0.0)
        wall_segment.rotate(y = -math.pi/2, z = -math.pi/2)
    for y in range(int(size_y / segment_size_y)):
        wall_segment = PassiveObject('environment_segments/wall_segment.blend', 'wall')
        wall_segment.translate(0.0,y * segment_size_y, 0.0)
        wall_segment.rotate(y = -math.pi/2)
    for x in range(int(size_x / segment_size_x)):
        wall_segment = PassiveObject('environment_segments/wall_segment.blend', 'wall')
        wall_segment.translate(x * segment_size_x, size_y, 0.0)
        wall_segment.rotate(y = -math.pi/2, z = -math.pi/2)
    for y in range(int(size_y / segment_size_y)):
        wall_segment = PassiveObject('environment_segments/wall_segment.blend', 'wall')
        wall_segment.translate(size_x, y * segment_size_y, 0.0)
        wall_segment.rotate(y = -math.pi/2)



    #creating the internal walls ... loaded from json file
    for y in range(2):
        wall_segment = PassiveObject('environment_segments/wall_segment1.blend', 'wall')
        wall_segment.translate(wall['x'],wall['y'], 0.0)
        wall_segment.rotate(y = wall["orientation"]* math.pi/180) # convert degree to rad


def create_tray_sensor(x, y, z, t_id, tray_type):
    tray_robot = FakeRobot()
    tray_robot.add_default_interface('ros')
    tray_robot.translate(x, y, z)

    tray_sensor = Proximity()
    tray_sensor.add_stream('ros', 'tray_sensor_publisher.TraySensorPublisher', tray_id = t_id, topic = '/warehouse/tray_sensors')
    if tray_type == "input":
        tray_sensor.properties(Range = 0.25, Track = 'Package')
    else:
        tray_sensor.properties(Range = 0.25, Track = 'Package')
    tray_sensor.frequency(2)
    tray_sensor.translate(0, 0, 0)

    tray_robot.append(tray_sensor)


def create_id_texts(tray, tray_id, x, y, z, orientation):
    # TODO: These below are all hard coded due to the auto assignment of trays regardless of their types. Those id assignment should be fixed !
    # if tray['type'] == 'input':
    #     tray_id_txt = str(tray_id)
    # if tray['type'] == 'output':
    #     tray_id_txt = str(tray_id)
    # if tray['type'] == 'storage':
    #     tray_id_txt = str(tray_id)
    # if tray['type'] == 'charging station':
    #     tray_id_txt = str(tray_id + 1)
    if tray_id > 20:
        tray_id = 20
    tray_id_txt = str(tray_id)
    tray_id_text = PassiveObject('tray_labels/tray_' + tray_id_txt + '.blend', 'Text')
    #print(tray_id_text)
    # put the text with an offset so that it is on the floor right in front of the tray
    if orientation == -90:
        y -= 0.50
        x -= 0.10
    elif orientation == 90:
        y += 0.50
        x += 0.10
    elif orientation == 180:
        x -= 0.50
        y += 0.10
    elif orientation == 0:
        x += 0.50
        y -= 0.10
    tray_id_text.translate(x, y, z)
    tray_id_text.rotate(0.0, 0.0, math.radians(orientation + 90))


def create_tray(tray_id, typeCnt, tray, simply_flag):
    # determine model
    if tray['type'] == 'input':
        tray_model_file = 'input_tray.blend'
        tray_model_name = 'InputTray'
    if tray['type'] == 'output':
        tray_model_file = 'output_tray.blend'
        tray_model_name = 'OutputTray'
    if tray['type'] == 'storage':
        tray_model_file = 'storage_tray.blend'
        tray_model_name = 'StorageTray'

    input_tray = PassiveObject(tray_model_file, tray_model_name)
    input_tray.translate(tray['x'], tray['y'], 0.0)
    #input_tray.translate(tray['x'], tray['y'], 0.0)
    input_tray.rotate(0.0, 0.0, math.radians(tray['orientation']))


    # add tray sensor
    create_tray_sensor(tray['x'], tray['y'], 0.28, tray_id, tray['type']) # 0.3

    # add ids of trays
    #create_id_texts(tray, typeCnt, tray['x'], tray['y'], 0, tray['orientation'])

def create_conveyor(map_config):
    for conveyor in map_config['human_robot_collaboration']['conveyors']:
        if conveyor['type'] == "L":
            conveyor1 = Conveyor(conveyor['id'] + '/first_part')
            conveyor1.translate(conveyor['location']['x'], conveyor['location']['y'], 0)
            conveyor1.rotate((conveyor['rotation'] * math.pi) / 180)

            conveyor2 = Conveyor(conveyor['id'] + '/second_part')
            conveyor2.translate(conveyor['location']['x'] + 0.75, conveyor['location']['y'] - 1.6,
                                0)  # -1.36
            conveyor2.rotate(z = ((conveyor['rotation'] * math.pi) / 180) + 1.57)
            conveyor3 = Conveyor(conveyor['id'] + '/third_part')
            conveyor3.translate(conveyor['location']['x'] + 0.75 + 1.9, conveyor['location']['y'] - 1.6,
                                0)  # -1.36
            conveyor3.rotate(((conveyor['rotation'] * math.pi) / 180) + 1.57)

            conveyor1.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')
            conveyor2.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')
            conveyor3.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')

        elif conveyor['type'] == "D":
            conveyor1 = ConveyorDistributor(conveyor['id'] + '/assembly_part')
            conveyor1.translate(conveyor['location']['x'], conveyor['location']['y'], 0)
            conveyor1.rotate(z=(conveyor['rotation'] * math.pi) / 180)
            conveyor2 = Conveyor(conveyor['id'] + '/printer_part') # TODO: remove the ID
            conveyor2.translate(conveyor['location']['x'], conveyor['location']['y'] - 2.25,
                                0)  # -1.36
            conveyor2.rotate(z = (conveyor['rotation'] * math.pi) / 180)
            conveyor1.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')
            conveyor2.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')

        elif conveyor['type'] == "Object_Inspection":
            conveyor1_x = conveyor['location']['x']
            conveyor1_y = conveyor['location']['y']

            offset1 = 1.60
            offset2 = offset1 + 1.65

            rotating_table_x = conveyor1_x
            rotating_table_y = conveyor1_y - offset1

            conveyor2_x = conveyor1_x
            conveyor2_y = conveyor1_y - offset2

            conveyor1 = ConveyorDistributor(conveyor['id'] + '/assembly_part')
            conveyor1.translate(conveyor1_x, conveyor1_y, 0)
            conveyor1.rotate(z=(conveyor['rotation'] * math.pi) / 180)

            rotating_table = RotatingTable(conveyor['id']+'/rotating_table')
            rotating_table.translate(rotating_table_x, rotating_table_y, 0.05)
            rotating_table.rotate(z = ((conveyor['rotation'] + 90) * math.pi) / 180)
            rotating_table.add_service('ros')

            kinect1 = Kinect()
            kinect1.depth_camera.add_interface('ros',
                                  topic=conveyor['id'] + '/kinect_top/depth')
            kinect1.video_camera.add_interface('ros',
                                  topic=conveyor['id'] + '/kinect_top/rgb')
            # TODO cleanup / find solution for multiple cameras
            # publish also to this topic to test openni
            if conveyor['id'] == "conveyor_1":
                kinect1.depth_camera.add_interface('ros',
                                      topic='/camera/depth',
                                      topic_suffix='/image_raw')
                kinect1.video_camera.add_interface('ros',
                                      topic='/camera/rgb',
                                      topic_suffix='/image_raw')
            kinect1.translate(0, -0.1, 1.7)
            kinect1.rotate(0, math.pi/2.0, 0)

            kinect2 = Kinect()
            kinect2.depth_camera.add_interface('ros',
                                 topic=conveyor['id'] + '/kinect_side/depth')
            kinect2.video_camera.add_interface('ros',
                                  topic=conveyor['id'] + '/kinect_side/rgb')
            kinect2.translate(0, -0.85, 0.7)
            kinect2.rotate(0, 0, math.pi/2.0)

            rotating_table.append(kinect1)
            rotating_table.append(kinect2)

            conveyor2 = Conveyor(conveyor['id'] + '/printer_part')
            conveyor2.translate(conveyor2_x, conveyor2_y, 0)
            conveyor2.rotate(z = (conveyor['rotation'] * math.pi) / 180)

            conveyor1.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')
            conveyor2.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')

        else:  # tip to tip connection of the belts
            conveyor1 = Conveyor(conveyor['id'] + '/first_part')
            conveyor1.translate(conveyor['location']['x'], conveyor['location']['y'], 0)
            conveyor1.rotate((conveyor['rotation'] * math.pi) / 180)

            conveyor2 = Conveyor(conveyor['id'] + '/second_part')
            conveyor2.translate(conveyor['location']['x'], conveyor['location']['y'] - 1.8, 0)
            conveyor2.rotate((conveyor['rotation'] * math.pi) / 180)
            conveyor1.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')
            conveyor2.add_overlay('ros', 'auto_warehouse.overlays.conveyor_overlay.ConveyorControlOverlay')
        # adding 3D printer objects on the scene
        adding_printer = PassiveObject('printer.blend', 'printer')
        adding_printer.translate(conveyor['location']['x'], conveyor['location']['y'] - 3.95, 0.60)
        adding_printer.rotate(z=(conveyor['rotation'] * math.pi) / 180)


def rotate_point(x, y, angle):
    xn = x * math.cos(angle) - y * math.sin(angle)
    yn = y * math.cos(angle) + x * math.sin(angle)
    return (xn, yn)

def create_charging_station(tray_id, typeCnt, tray):
    # create charging station object
    charging_station = PassiveObject('charging_station.blend', 'ChargingStation')
    charging_station.translate(tray['x'], tray['y'], 0.0)
    angle = math.radians(tray['orientation'])
    charging_station.rotate(0.0, 0.0, angle)
    #create_id_texts(tray, typeCnt, tray['x'], tray['y'], 0, tray['orientation'])

    # define charging zone
    charging_zone = Zone(type = 'Charging')
    charging_zone.size = [0.5, 0.5, 1.0]

    # compute offset from tray center also taking rotation into account
    xoff = 0.75
    yoff = 0.0
    xoffn, yoffn = rotate_point(xoff, yoff, angle)

    charging_zone.translate(tray['x'] + xoffn, tray['y'] + yoffn, 0.0)
    # not supported by MORSE
    #charging_zone.rotate(0.0, 0.0, angle)

def create_package_pool(pool):
    package_pool = PassiveObject('package_pool_container.blend', 'PackagePoolContainer')
    package_pool.translate(pool['location']['x'], pool['location']['y'], pool['location']['z'])

def setup_warehouse_map(map_config, robot_config, simplfy_flag):
    # create package pool container
    create_package_pool(map_config['package_pool'])

    # create floor to cover warehouse size
    create_floor(map_config['map']['width'], map_config['map']['height'])

    if (not simplfy_flag): # the flag is only used to put walls or not, but significantly saves computation

    # create walls
        for wall in enumerate(map_config['walls']):
            create_walls(wall[1], map_config['map']['width'], map_config['map']['height'])

    # add conveyor belts
    create_conveyor(map_config)

    # add human and robot collaborators on conveyor belt
    # TODO: this function will be updated to include jido kuka?
    add_human_robot.create_human_and_robot(map_config)

    # create trays and charging stations (which are considered special type of trays)
    inputCnt = 0
    outputCnt = 0
    storageCnt = 0
    chargingCnt = 0

    for tray_id, tray in enumerate(map_config['trays']):
        if tray['type'] == 'charging station':
            chargingCnt += 1
            create_charging_station(tray_id, chargingCnt, tray)

        if tray['type'] == 'input':
            inputCnt += 1
            create_tray(tray_id, inputCnt, tray, simplfy_flag)
        if tray['type'] == 'output':
            outputCnt += 1
            create_tray(tray_id, outputCnt, tray, simplfy_flag)
        if tray['type'] == 'storage':
            storageCnt += 1
            create_tray(tray_id, storageCnt, tray, simplfy_flag)
        # else:
        #     create_tray(tray_id, tray, simplfy_flag)


    # create robots
    for robot in map_config['robots']:
        type = robot['type']
        if type == 'Pioneer P3-DX':
            pioneer_robot.create_robot(robot['name'], robot['idle_position']['x'], robot['idle_position']['y'], math.radians(robot['idle_position']['orientation']), robot_config[type])
            continue
        if type == 'Pioneer P3-DX Light':
            pioneer_light_robot.create_robot(robot['name'], robot['idle_position']['x'], robot['idle_position']['y'], math.radians(robot['idle_position']['orientation']), robot_config[type])
            continue
    # TODO: Once the kuka integration is complete with navigation and grasping, then this part will be added under map_config['robots']
    for robot in map_config['kuka_robots']:
        type = robot['type']
        if type == 'Kuka Jido':
            kuka_jido.create_robot(robot['name'], robot['idle_position']['x'], robot['idle_position']['y'], math.radians(robot['idle_position']['orientation']), robot_config[type])
            continue
        print('Unknown robot type. Skip robot creation.')

