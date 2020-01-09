from math import *
from Robot_Simulator_V3 import twoRoomsWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import RobotMovement


if __name__ == '__main__':
    """ main """
    myWorld = twoRoomsWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()
    myWorld.setRobot(myRobot, [16, 12, 0])
    path = [[3, 10], [3, 2.5], [7, 2.5], [10, 3], [10, 10], [16, 10], [16, 12]]
    myRobot.drivePath(0.2, path)

    myWorld.close()
