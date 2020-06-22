from math import *
from Robot_Simulator_V3 import emptyWorld, labyrinthWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import RobotMovement


def exercisea():
    #myRobot.wander(1)
    myRobot.followWall(0.5, 0.7)


if __name__ == '__main__':
    """ main """
    # Roboter in einer Welt positionieren:
    myWorld = labyrinthWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()

    myWorld.setRobot(myRobot, [10, 5.5, pi / 2])

    exercisea()
    myWorld.close()
