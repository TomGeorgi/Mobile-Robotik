from math import *
from Robot_Simulator_V3 import simpleWorld
from Robot_Simulator_V3 import RobotMovement
from Robot_Simulator_V3.particle_filter_pose_estimator import ParticleFilterPoseEstimator


if __name__ == '__main__':
    """ main """
    myWorld = simpleWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()

    myGrid = myWorld.getDistanceGrid()
    pfpe = ParticleFilterPoseEstimator(myRobot)
    pfpe.initialize((2, 2, pi), (6, 6, 0), n=10)
    pfpe.getCovariance()

    myWorld.setRobot(myRobot, [4, 4, pi/2])
    myRobot.curveDrive(0.2, 4.5, -180)

    myWorld.close()
