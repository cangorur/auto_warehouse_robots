#!/usr/bin/env python

import json
import copy

from auto_warehousing.msg import Tray

class WarehouseConfigGenerator(object):

    trayPrototype = {
        'type' : 'storage',
        'x' : 0.0,
        'y' : 0.0,
        'orientation' : 0.0,
        'max_load' : 50.0,
        'package_type': 0
    }

    trayGeometry = {
        'width' : 0.5,
        'height' : 0.5
    }

    def __init__(self, mapWidth, mapHeight):
        self.trays = []
        self.robots = []
        self.robotId = 1
        self.mapWidth = mapWidth
        self.mapHeight = mapHeight

    def generateTray(self, x, y, orientation):
        t = copy.deepcopy(WarehouseConfigGenerator.trayPrototype)

        t['x'] = x
        t['y'] = y
        t['orientation'] = orientation

        self.trays.append(t)

    def generateTrayRow(self, x, y, n, facingUp = True):
        for i in range(n):
            if facingUp:
                orientation = 90.0
            else:
                orientation = -90.0
            self.generateTray(x + i * WarehouseConfigGenerator.trayGeometry['width'], y, orientation)

    def generateTrayColumn(self, x, y, n, facingLeft = True):
        for i in range(n):
            if facingLeft:
                orientation = 180.0
            else:
                orientation = 0.0
            self.generateTray(x, y + i * WarehouseConfigGenerator.trayGeometry['height'], orientation)

    def generateHBar(self, x, y, n):
        self.generateTrayRow(x, y, n, False)
        self.generateTrayRow(x, y + WarehouseConfigGenerator.trayGeometry['height'], n, True)

    def generateVBar(self, x, y, n):
        self.generateTrayColumn(x, y, n, True)
        self.generateTrayRow(x + WarehouseConfigGenerator.trayGeometry['width'], y, n, False)
        
    def generateRobot(self, startx, starty, type_name):
        robot = {}
        robot['name'] = 'robot_' + str(self.robotId)
        self.robotId += 1
        robot['type'] = type_name
        robot['idle_position'] = {}
        robot['idle_position']['x'] = startx
        robot['idle_position']['y'] = starty
        robot['idle_position']['orientation'] = 0
        robot['start_x'] = startx
        robot['start_y'] = starty
        robot['start_orientation'] = 0
        
        self.robots.append(robot)

    def saveConfig(self, file):
        config = {
            'map' : {
                'width' : self.mapWidth,
                'height' : self.mapHeight
            },
            'package_pool' : {
                'location' : {
                    "x" : -10.0,
                    "y" : 4.0,
                    "z" : 0.0
                },
                "relative_drop_location" : {
                    "x" : 0.0,
                    "y" : -1.5,
                    "z" : 2.0
                },
                "relative_stacking_area" : {
                    "x1" : -2,
                    "y1" : 1,
                    "x2" : 2,
                    "y2" : 2.3,
                    "z" : 0.3
                }
            },
            'tray_geometry' : {
                'width' : WarehouseConfigGenerator.trayGeometry['width'],
                'height' : WarehouseConfigGenerator.trayGeometry['height']
            },
            'trays' : self.trays,
            'robots' : self.robots
        }

        with open(file, 'w') as f:
            json.dump(config, f, indent = 4)

def main():
    gen = WarehouseConfigGenerator(18, 11)
    
    #gen.generateRobot(1, 1, 'Pioneer P3-DX')

    # left part

    gen.generateTrayRow(0.75, 2.25, 4, True)
    
    gen.generateTrayRow(0.75, 4.75, 4, False)
    gen.generateTrayRow(0.75, 5.25, 4, True)
    
    gen.generateTrayRow(0.75, 7.75, 4, False)
    gen.generateTrayRow(0.75, 8.25, 4, True)
    
    gen.generateTrayRow(0.75, 10.75, 4, False)
    
    # lower part
    
    gen.generateTrayColumn(7.25, 0.75, 4, False)
    
    gen.generateTrayColumn(11.75, 0.75, 4, True)
    
    # middle part
    
    gen.generateTrayColumn(7.25, 5.25, 6, False)
    
    gen.generateTrayColumn(11.75, 5.25, 6, True)
    
    # upper part
    
    gen.generateTrayRow(7.25, 8.25, 20, True)
    
    gen.generateTrayRow(7.25, 10.75, 20, False)
    
    # charging stations
    
    gen.generateTray(12.25, 5.25, 0)
    gen.generateTray(12.25, 6.25, 0)
    gen.generateTray(12.25, 7.25, 0)
    
    gen.generateTray(16.75, 5.25, 180)
    gen.generateTray(16.75, 6.25, 180)
    gen.generateTray(16.75, 7.25, 180)
    
    gen.generateTray(12.25, 1.25, 0)
    gen.generateTray(12.25, 2.25, 0)
    
    gen.generateTray(16.75, 1.25, 180)
    gen.generateTray(16.75, 2.25, 180)
    
    #gen.generateHBar(5.25, 4.0, 20)
    #gen.generateTrayRow(5.25, 9.75, 20, False)
    
    #gen.generateTrayRow(5.25, 10.25, 20, True)
    #gen.generateHBar(5.25, 14.0, 20)
    #gen.generateTrayRow(5.25, 19.75, 20, False)

    gen.saveConfig('large_warehouse_config_gen.json')

if __name__ == '__main__':
    main()
