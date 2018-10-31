from morse.builder import *
from auto_smart_factory_sim.builder.sensors import Custombattery

def create_robot(name, x, y, orientation, robot_config):
    robot = Jido(name)
    robot.add_default_interface('ros')
    robot.properties(GroundRobot=True)

    # set start pose
    robot.translate(x, y, 0.0)
    robot.rotate(0.0, 0.0, orientation)
    
    # add actuator

    kuka = KukaLWR()
    robot.append(kuka)
    kuka.translate(x=0.1850, y=0.2000, z=0.9070)
    kuka.rotate(x=1.5708, y=1.5708)
 
    kuka.add_interface('socket')
    kuka.add_interface('ros', method="morse.middleware.ros.kuka_jointstate.JointStateReader" )

    motion = MotionXYW()
    motion.properties(ControlType = 'Position')
    robot.append(motion)
    motion.add_interface('ros', topic='/cmd_vel')

    # add battery
    battery = Custombattery()
    battery.frequency(1)
    battery.add_overlay('ros', 'battery_overlay.RandomInitBatteryOverlay')
    battery.properties(DischargingRate=robot_config['discharging_rate'], ChargingRate=robot_config['charging_rate'])
    battery.add_stream('ros', 'morse.middleware.ros.battery.Float32Publisher')
    robot.append(battery)

    # Add a pose sensor
    pose = Pose()
    robot.append(pose)
    pose.add_interface('ros', topic="/jido/pose")
    

    # add laser scanner
    hokuyo = Hokuyo()
    hokuyo.translate(x=0.275, z=0.252)
    robot.append(hokuyo)
    hokuyo.properties(Visible_arc = False)
    hokuyo.properties(laser_range = 30.0)
    hokuyo.properties(resolution = 1.0)
    hokuyo.properties(scan_window = 180.0)
    hokuyo.create_laser_arc()
    hokuyo.add_interface('ros', topic='/base_scan')
    
"""    label = PassiveObject('robot_label_pioneer.blend', 'Label')
    label.translate(0.0, 0.0, 0.15)
    label.rotate(0.0, 0.0, math.pi / 2)
    robot.append(label) """
    
