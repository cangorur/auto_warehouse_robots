from morse.builder import *
from auto_smart_factory_sim.builder.sensors import Custombattery

def create_robot(name, x, y, orientation, robot_config):
    robot = Pioneer3DX(name)
    robot.add_default_interface('ros')
    robot.properties(GroundRobot = True)

    # set start pose
    robot.translate(x, y, 0.0)
    robot.rotate(0.0, 0.0, orientation)

    # add actuator
    motion = MotionVWDiff()
    robot.append(motion)

    # add gripper
    gripper = Gripper()
    gripper.translate(-0.0, 0.0, 0.23) #0.23
    gripper.rotate(-0.0, math.pi / 2.0, 0.0)
    gripper.properties(Angle=0.0, Distance=20.0)
    gripper.add_overlay('ros', 'gripper_overlays.LoadUnloadOverlay')
    robot.append(gripper)

    # Add a pose sensor
    pose = Pose()
    pose.frequency(40)
    robot.append(pose)

    # add battery
    battery = Custombattery()
    battery.frequency(1)
    battery.add_overlay('ros', 'battery_overlay.RandomInitBatteryOverlay')
    #battery.properties(DischargingRate = robot_config['discharging_rate'], ChargingRate = robot_config['charging_rate'])
    battery.properties(DischargingRate = robot_config['discharging_rate'], ChargingRate = robot_config['charging_rate'], MotorDrainingRate = robot_config['motor_draining_rate'])
    battery.add_stream('ros', 'morse.middleware.ros.battery.Float32Publisher')
    robot.append(battery)

    # add laser scanner
    hokuyo = Hokuyo('laser_scanner')
    #hokuyo.translate(0.22, 0.0, 0.06)
    hokuyo.translate(0.2, 0.0, 0.0)
    hokuyo.properties(Visible_arc = False)
    hokuyo.properties(laser_range = 3.0)
    hokuyo.properties(scan_window = 210)
    hokuyo.properties(resolution = 2.0)
    hokuyo.frequency(3.0)
    robot.append(hokuyo)

    label = PassiveObject('robot_labels/label_pioneer_light_' + name + '.blend', 'Label')
    label.translate(0.0, 0.0, 0.15)
    label.rotate(0.0, 0.0, -1 * math.pi / 2)
    robot.append(label)
