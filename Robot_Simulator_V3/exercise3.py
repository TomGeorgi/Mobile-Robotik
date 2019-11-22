from math import *
from Robot_Simulator_V3 import emptyWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import RobotMovement

# Roboter in einer Welt positionieren:
myWorld = emptyWorld.buildWorld()
myRobot = RobotMovement.RobotMovement()
myWorld.setRobot(myRobot, [10, 5.5, pi / 2])

myRobot.rectangle(0.2, 1)
myRobot.laneChange(0.2, 1)

# Simulation schliessen:
myWorld.close()