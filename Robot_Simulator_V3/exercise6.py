from math import *

from PoseEstimator.plotUtilities import plotPositionCovariance
from Robot_Simulator_V3 import simpleWorld
from Robot_Simulator_V3 import RobotMovement
from Robot_Simulator_V3 import graphics
from Robot_Simulator_V3.World import World
from Robot_Simulator_V3.particle_filter_pose_estimator import ParticleFilterPoseEstimator
import matplotlib.pyplot as plt


def drawParticles(myWorld: World, particles):
    # myWorld.undrawLines()
    polylines = []
    for particle in particles:
        p1_x = particle[0]
        p1_y = particle[1]

        delta_y = 0.1 * sin(particle[2])
        delta_x = 0.1 * cos(particle[2])

        p2_x = p1_x + delta_x
        p2_y = particle[1] + delta_y

        polylines.append(((p1_x, p1_y), (p2_x, p2_y)))
    myWorld.drawPolylines(polylines)


def a():
    myWorld = simpleWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()

    pfpe = ParticleFilterPoseEstimator(myRobot)
    pfpe.initialize((2, 2, pi), (6, 6, 0), n=10)
    drawParticles(myWorld, pfpe.particles)

    myWorld.setRobot(myRobot, [4, 4, pi / 2])

    myRobot.move([1, 0.3])
    pfpe.integrateMovement([1, 0.3])
    drawParticles(myWorld, pfpe.particles)

    myRobot.move([1, 0.3])
    pfpe.integrateMovement([1, 0.3])
    drawParticles(myWorld, pfpe.particles)

    myRobot.move([1, 0.3])
    pfpe.integrateMovement([1, 0.3])
    drawParticles(myWorld, pfpe.particles)

    myRobot.move([1, 0.3])
    pfpe.integrateMovement([1, 0.3])
    drawParticles(myWorld, pfpe.particles)
    # myRobot.curveDrive(0.2, 4.5, -180)

    myWorld.close()


def b():
    myWorld = simpleWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()
    myWorld.setRobot(myRobot, [4, 4, pi / 2])

    myGrid = myWorld.getDistanceGrid()
    pfpe = ParticleFilterPoseEstimator(myRobot)
    pfpe.initialize((3, 3, pi), (7, 7, 0), n=200)
    drawParticles(myWorld, pfpe.particles)
    pose = pfpe.getPose()

    point = graphics.Point(pose[0], pose[1])
    c = graphics.Circle(point, 0.1)
    c.setFill(("yellow"))
    c.draw(myWorld.win)

    pfpe.integrateMeasurement(myRobot.sense(), myRobot.getSensorDirections(), myGrid)
    drawParticles(myWorld, pfpe.particles)
    pose = pfpe.getPose()

    point = graphics.Point(pose[0], pose[1])

    c = graphics.Circle(point, 0.1)
    c.setFill(("blue"))
    c.draw(myWorld.win)

    myWorld.close()


def c():
    myWorld = simpleWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()
    myWorld.setRobot(myRobot, [4, 4, pi / 2])
    myWorld._showPathHistory = True

    myGrid = myWorld.getDistanceGrid()
    pfpe = ParticleFilterPoseEstimator(myRobot)
    pfpe.initialize((3, 3, pi), (15, 10, 0), n=500)
    drawParticles(myWorld, pfpe.particles)
    pose = pfpe.getPose()
    point = graphics.Point(pose[0], pose[1])
    c = graphics.Circle(point, 0.1)
    c.setFill(("yellow"))
    c.draw(myWorld.win)

    motionCircle = curveDrive(0.8, 4, -180)

    idx = 0
    truePose = []
    trueRobotPose = myWorld.getTrueRobotPose()
    truePose.append((trueRobotPose[0], trueRobotPose[1]))
    for motion in motionCircle:
        myRobot.move(motion)
        trueRobotPose = myWorld.getTrueRobotPose()
        truePose.append((trueRobotPose[0], trueRobotPose[1]))
        pfpe.integrateMovement(motion)
        pfpe.integrateMeasurement(myRobot.sense(), myRobot.getSensorDirections(), myGrid)
        drawParticles(myWorld, pfpe.particles)
        pose = pfpe.getPose()

        if idx % 10 == 0:
            cov_x_y, sigma_theta = pfpe.getCovariance()
            plotPositionCovariance((pose[0], pose[1]), cov_x_y)

        idx += 1

        point = graphics.Point(pose[0], pose[1])

        c.undraw()
        c = graphics.Circle(point, 0.1)
        c.setFill(("blue"))
        c.draw(myWorld.win)

    for _ in range(0, len(truePose)):
        x = [i[0] for i in truePose]
        y = [i[1] for i in truePose]
        plt.plot(x, y)
    plt.show()

    myWorld.close()


def d():
    myWorld = simpleWorld.buildWorld()
    myRobot = RobotMovement.RobotMovement()
    myWorld.setRobot(myRobot, [4, 4, pi / 2])
    myWorld._showPathHistory = True
    myWorld.addDynObstacleLine(7, 4, 7, 7)

    myGrid = myWorld.getDistanceGrid()
    myGrid.drawGrid()
    pfpe = ParticleFilterPoseEstimator(myRobot)
    pfpe.initialize((3, 3, pi), (7, 7, 0), n=200)
    drawParticles(myWorld, pfpe.particles)
    pose = pfpe.getPose()
    point = graphics.Point(pose[0], pose[1])
    c = graphics.Circle(point, 0.1)
    c.setFill(("yellow"))
    c.draw(myWorld.win)

    motionCircle = curveDrive(0.2, 4, -180)

    circle = c
    for motion in motionCircle:
        myRobot.move(motion)
        pfpe.integrateMovement(motion)
        pfpe.integrateMeasurement(myRobot.sense(), myRobot.getSensorDirections(), myGrid)
        drawParticles(myWorld, pfpe.particles)
        pose = pfpe.getPose()

        cov_x_y, sigma_theta = pfpe.getCovariance()
        plotPositionCovariance((pose[0], pose[1]), cov_x_y)

        point = graphics.Point(pose[0], pose[1])

        c.undraw()
        c = graphics.Circle(point, 0.1)
        c.setFill(("blue"))
        c.draw(myWorld.win)

    myWorld.close()


def curveDrive(v, r, theta):
    w = v / r

    omega = w

    theta_rad = theta * pi / 180

    if theta >= 0:
        n = round((theta_rad / omega) * 10)

        # Definiere Folge von Bewegungsbefehle:
        motionCircle = [[v, omega] for i in range(n)]
    else:
        n = -ceil((theta_rad / omega) * 10)

        # Definiere Folge von Bewegungsbefehle:
        motionCircle = [[v, -omega] for i in range(n)]

    return motionCircle


if __name__ == '__main__':
    """ main """
    # a()
    # b()
    c()
    # d()
