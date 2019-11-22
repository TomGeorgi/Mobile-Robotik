# class Robot.
#
# This class define methods to move a robot
# and to sense the world with a Laser Scanner.
# Distance measurements to landmarks are available.
# Additionally boxes can be sensed, picked up and placed.
#
# The robot does not know its pose!
#
# O. Bittel; 13.09.2019


from math import *
import numpy as np
import random

from Robot_Simulator_V3.Robot import Robot


class RobotMovement(Robot):
    # --------
    # init: creates robot.
    #
    def __init__(self):
        self._size = 0.5  # robot diameter
        self._T = 0.1  # time step
        self._world = None  # robot's world; is set by setWorld()

        # Motion noise parameter:
        self._k_d = 0  # 0.05 * 0.05  # velocity noise parameter = 0.05m*0.05m / 1m
        self._k_theta = 0  # (5.0 * 5.0 / 360.0) * (
        # pi / 180.0)  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
        self._k_drift = 0  # (2.0 * 2.0) / 1.0 * (pi / 180.0) ** 2  # drift noise parameter = 2deg*2deg / 1m
        self._maxSpeed = 1.0  # maximum speed
        self._maxOmega = pi  # maximum rotational speed

        # SigmaMotion
        self._SigmaMotion = np.zeros((2, 2))

        # Laser Sensor (x-axis is in forward direction):
        self._numberOfBeams = 28
        self._viewAngle = 270.0
        dTheta = self._viewAngle / (self._numberOfBeams - 1)
        self._sensorDirections = [(-135.0 + dTheta * i) * (pi / 180.0) for i in range(self._numberOfBeams)]
        self._maxSenseValue = 5.0  # Maximum sensor value for each sensor beam
        self._sensorNoise = 0.01  # standard deviation of distance measurement for 1m

        # Landmark measurement
        self._landmarks = []  # landmark positions
        self._senseNoiseLandmarks = 0.1  # standard deviation of distance measurement

        # Pick and Place Boxes:
        self._boxSensor = True
        self._boxMinSenseValue = 0.2  # Maximum sensor value for each sensor beam
        self._boxMaxSenseValue = 5.0  # Maximum sensor value for each sensor beam
        self._boxDistSensorNoise = 0.01  # standard deviation of distance measurement for 1m
        self._boxAngleSensorNoise = 1 * (pi / 180)  # standard deviation of angle measurement in rad
        self._boxPlace_x = 0.25  # box place in local x axis
        self._boxPlace_y = 0.0  # box place in local y axis
        self._boxPickUp_x_min = 0.2  # box pick up area in local x axis
        self._boxPickUp_x_max = 0.5  # box pick up area in local x axis
        self._boxPickUp_y_min = -0.1  # box pick up area in local y axis
        self._boxPickUp_y_max = 0.1  # box pick up area in local y axis
        self._boxPickedUp = False
        self._boxInPickUpPosition = False

    def curveDrive(self, v, r, theta):
        if v > self._maxSpeed:
            v = 1

        if self.isclose(r, 0) | self.isclose(v, 0):
            w = inf
        else:
            w = v / r

        omega = w * pi / 180
        if omega > self._maxOmega:
            omega = self._maxOmega

        theta_rad = theta * pi / 180

        if theta >= 0:
            n = round((theta_rad / omega) * 10)

            # Definiere Folge von Bewegungsbefehle:
            motionCircle = [[v, omega] for i in range(n)]
        else:
            n = -ceil((theta_rad / omega) * 10)

            # Definiere Folge von Bewegungsbefehle:
            motionCircle = [[v, -omega] for i in range(n)]

        # Bewege Roboter
        for t in range(n):
            # Bewege Roboter
            print(motionCircle[t])
            self.move(motionCircle[t])

    def straightDrive(self, v, l):
        n = round((l / v) * 10)
        motionLine = [[v, 0] for i in range(n)]
        # Bewege Roboter
        for t in range(n):
            # Bewege Roboter
            print(motionLine[t])
            self.move(motionLine[t])

    def rectangle(self, v, l):
        self.straightDrive(v, l)
        self.curveDrive(0, 0, 90)
        self.straightDrive(v, l)
        self.curveDrive(0, 0, 90)
        self.straightDrive(v, l)
        self.curveDrive(0, 0, 90)
        self.straightDrive(v, l)
        self.curveDrive(0, 0, 90)

    def laneChange(self, v, w):
        self.straightDrive(v, 2)
        self.curveDrive(v, 0.1, 45)
        self.straightDrive(v, w)
        self.curveDrive(v, 0.1, -45)
        self.straightDrive(v, 2)

    def isclose(self, a, b, rel_tol=1e-09, abs_tol=0.0):
        return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

    def followLine(self, p1, p2):
        p1_x, p1_y = p1
        p2_x, p2_y = p2

        vBase = np.array([p2_x-p1_x, p2_y - p1_y, 0])

        pos = self._world.getTrueRobotPose()
        pos_x = pos[0]
        pos_y = pos[1]
        ori = pos[2]
        print(vBase)

        vDistance = np.cross(vBase, np.array([pos_x-p1_x, pos_y - p1_y, 0]))
        print(vDistance)
        distance = sqrt(pow(vDistance[0], 2) + pow(vDistance[1], 2) + 0)
        print(distance)



    def gotoGlobal(self, v, p, tol):
        pos = self._world.getTrueRobotPose()
        px, py = p
        posx = pos[0]
        posy = pos[1]
        ori = pos[2]

        delta_x = px - posx
        delta_y = py - posy

        distance = sqrt(pow(delta_x, 2) + pow(delta_y, 2))

        # print(distance)

        if distance < tol:
            return

        theta = np.arctan2(delta_y, delta_x)

        delta_theta = theta - ori

        while delta_theta < -pi:
            delta_theta = delta_theta + (2 * pi)

        while delta_theta > pi:
            delta_theta = delta_theta - (2 * pi)

        print(delta_theta)
        K_w = 0.1

        # w = min(self._maxOmega, K_w * delta_theta)
        w = delta_theta
        self.move([v, w])
        self.gotoGlobal(v, p, tol)
