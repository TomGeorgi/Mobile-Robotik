from math import *
from Robot_Simulator_V3 import emptyWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import RobotMovement



def exercisea():
    myRobot.rectangle(0.2, 1)
    myRobot.laneChange(0.2, 1)

def exerciseb():
    myRobot.followLine((7, 3.5), (12, 3.5))

if __name__ == '__main__':
    """ main """
    # Roboter in einer Welt positionieren:
    myWorld = emptyWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()
    myWorld.setRobot(myRobot, [10, 5.5, pi / 5])

    exerciseb()
    # Simulation schliessen:
    myWorld.close()
