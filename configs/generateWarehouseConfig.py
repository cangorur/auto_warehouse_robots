#!/usr/bin/env python
import rospy

import json
import copy

class WarehouseConfigGenerator(object):

    trayPrototype = {
        'type' : 'storage',
        'x' : 0.0,
        'y' : 0.0,
        'orientation' : 0.0,
        'max_load' : 50.0,
        'package_type': 0
    }

    trayPrototype2 = {
        'type' : 'charging station',
        'x' : 0.0,
        'y' : 0.0,
        'orientation' : 0.0,
        'max_load' : 70.0,
        'package_type': 0
    }
    
    trayPrototype3 = {
        'type' : 'input',
        'x' : 0.0,
        'y' : 0.0,
        'orientation' : 0.0,
        'max_load' : 70.0,
        'package_type': 0
    }

    trayPrototype4 = {
        'type' : 'output',
        'x' : 0.0,
        'y' : 0.0,
        'orientation' : 0.0,
        'max_load' : 70.0,
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



    def generateChargingTray(self, x, y, orientation):
        t = copy.deepcopy(WarehouseConfigGenerator.trayPrototype2)

        t['x'] = x
        t['y'] = y
        t['orientation'] = orientation

        self.trays.append(t)


    def generateInputTray(self, x, y, orientation):
        t = copy.deepcopy(WarehouseConfigGenerator.trayPrototype3)

        t['x'] = x
        t['y'] = y
        t['orientation'] = orientation

        self.trays.append(t)
    
    def generateOutputTray(self, x, y, orientation):
        t = copy.deepcopy(WarehouseConfigGenerator.trayPrototype4)

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

    def generateInputTrayColumn(self, x, y, n, facingLeft = True):
        for i in range(n):
            if facingLeft:
                orientation = 180.0
            else:
                orientation = 0.0
            self.generateInputTray(x, y + i * WarehouseConfigGenerator.trayGeometry['height'], orientation)

    def generateOutputTrayColumn(self, x, y, n, facingLeft = True):
        for i in range(n):
            if facingLeft:
                orientation = 180.0
            else:
                orientation = 0.0
            self.generateOutputTray(x, y + i * WarehouseConfigGenerator.trayGeometry['height'], orientation)

    def generateHBar(self, x, y, n):
        self.generateTrayRow(x, y, n, False)
        self.generateTrayRow(x, y + WarehouseConfigGenerator.trayGeometry['height'], n, True)

    def generateVBar(self, x, y, n):
        self.generateTrayColumn(x, y, n, True)
        self.generateTrayRow(x + WarehouseConfigGenerator.trayGeometry['width'], y, n, False)
        
    def generateRobot(self, startx, starty, robotId, type_name):
        robot = {}
        robot['name'] = 'robot_' + str(robotId)
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
                'height' : self.mapHeight,
            	"margin" : 0.5,
	            "theta_star_resolution" : 0.5,
	            "mergeObstacles" : True
            },
            'package_pool' : {
                'location' : {
                    "x" : -5.0,
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
            "human_robot_collaboration": {
            "conveyors": [],
            "humans": [],
            "pr2_robots": []
            },
            'trays' : self.trays,
            'robots' : self.robots,
            'kuka_robots' : []
        }

        with open(file, 'w') as f:
            json.dump(config, f, indent = 4)

def main():

    trayWidth = 0.5
    mapWidth = 18
    mapHeight = 16
    numberOfTraysInARow = 5
    robotRadius = 0.25 # Same as in robot_config
    robotSize = 2*robotRadius
    
    numberofChargingStations = 2
    numberofInputTrays = 12
    numberOfRows = 3
    numberOfRobots = 4

    midPointWidth = mapWidth /2
    midPointHeight = mapHeight /2
    spaceBetweenRows = robotSize *2 
    spaceBetweenCols = robotSize * 5 
    spaceBetweenCS = trayWidth * 4
    rowLength = trayWidth * numberOfTraysInARow


    gen = WarehouseConfigGenerator(mapWidth, mapHeight)
    
    # Hack for shifiting
    rowOffset = 6
    columnOffset = 3
    distBetwChgandRows = 2
    topOffset = 6
    bottomOffset = 1

    for k in range(1,numberOfRobots+1):
        gen.generateRobot(k+4,2,k, 'Pioneer P3-DX')
        gen.generateRobot(k+4+4,2,k+4, 'Pioneer P3-DX Light')



    for i in range (numberOfRows):
        
        # Row i,0

        gen.generateTrayRow(midPointWidth-(spaceBetweenRows+rowLength), spaceBetweenCols * i + columnOffset, numberOfTraysInARow, False)
        gen.generateTrayRow(midPointWidth-(spaceBetweenRows+rowLength), spaceBetweenCols * i + columnOffset+ trayWidth, numberOfTraysInARow, True)

        # Row i,1

        gen.generateTrayRow(midPointWidth+(spaceBetweenRows), spaceBetweenCols* i +columnOffset, numberOfTraysInARow, False)
        gen.generateTrayRow(midPointWidth+(spaceBetweenRows), spaceBetweenCols * i + columnOffset+ trayWidth, numberOfTraysInARow, True)


    # Charging Stations Top
    gen.generateChargingTray((midPointWidth - spaceBetweenCS), bottomOffset, 90.0)
    gen.generateChargingTray((midPointWidth - spaceBetweenCS) - 3 * trayWidth, bottomOffset, 90.0)

    gen.generateChargingTray((midPointWidth + spaceBetweenCS) , bottomOffset, 90.0)
    gen.generateChargingTray((midPointWidth + spaceBetweenCS) + 3 * trayWidth , bottomOffset, 90.0)

    # Charging Stations Bottom
    gen.generateChargingTray((midPointWidth - spaceBetweenCS) - 3 * trayWidth , mapHeight - topOffset, 180.0)
    gen.generateChargingTray((midPointWidth - spaceBetweenCS)  , mapHeight - topOffset, 180.0)

    gen.generateChargingTray((midPointWidth + spaceBetweenCS) + 3 * trayWidth, mapHeight - topOffset, 180.0)
    gen.generateChargingTray((midPointWidth + spaceBetweenCS), mapHeight - topOffset, 180.0)


    
    gen.generateInputTrayColumn(3, 4.5, 5, False)
    gen.generateInputTrayColumn(2.5, 4.5, 5, True)

    gen.generateOutputTrayColumn(14, 4.5, 5, True)
    gen.generateOutputTrayColumn(14, 4.5, 5, False)

    # gen.generateHBar(5.25, 4.0, 20)
    #gen.generateTrayRow(5.25, 9.75, 20, False)
    
    #gen.generateTrayRow(5.25, 10.25, 20, True)
    #gen.generateHBar(5.25, 14.0, 20)
    #gen.generateTrayRow(5.25, 19.75, 20, False)

    gen.saveConfig('dummy_test_config.json')

if __name__ == '__main__':
    main()
