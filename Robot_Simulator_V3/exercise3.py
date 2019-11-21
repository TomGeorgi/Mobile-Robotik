from math import *
from Robot_Simulator_V3 import emptyWorld
from Robot_Simulator_V3 import Robot

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = Robot.Robot()
myWorld.setRobot(myRobot, [10, 5.5, pi / 2])

myRobot.curveDrive(0.1, 0.05, 180)

# Simulation schliessen:
myWorld.close()