from math import *
from Robot_Simulator_V3 import emptyWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import RobotMovement


def exercisea():
    myRobot.rectangle(0.1, 1)
    myRobot.laneChange(0.1, 1)


def exerciseb():
    # myWorld.addLine(7, 20, 12, 20)
    # myRobot.followLine((3, 10), (20, 10))
    myRobot.followPolyline(0.3, [(3, 10), (5, 15), (7, 10), (9, 15), (11, 10)])
    # myRobot.gotoGlobal(0.3, (3, 10), 0.4)

if __name__ == '__main__':
    """ main """
    # Roboter in einer Welt positionieren:
    myWorld = emptyWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()

    myWorld.setRobot(myRobot, [10, 5.5, pi / 2])
    myWorld._showPathHistory = True
    myWorld.drawPolyline(((3, 10), (20, 10)))

    exerciseb()
    # exercisea()
    # Simulation schliessen:
    myWorld.close()
