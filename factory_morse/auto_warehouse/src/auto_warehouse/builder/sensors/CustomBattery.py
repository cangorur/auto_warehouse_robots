from morse.builder.creator import SensorCreator

class Custombattery(SensorCreator):
    _classpath = "auto_warehouse.sensors.CustomBattery.Custombattery"
    _blendname = "CustomBattery"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

