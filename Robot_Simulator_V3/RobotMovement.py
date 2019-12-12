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
from Robot_Simulator_V3 import geometry, sensorUtilities


class RobotMovement(Robot):
    # --------
    # init: creates robot.
    #
    def __init__(self):
        super(RobotMovement, self).__init__()

    def curveDrive(self, v, r, theta):
        if v > self._maxSpeed:
            v = 1

        if self.isclose(r, 0) | self.isclose(v, 0):
            w = inf
        else:
            w = v / r

        omega = w
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

        pos = self._world.getTrueRobotPose()
        pos_x = pos[0]
        pos_y = pos[1]
        while abs(sqrt(pow(p2_x - pos_x, 2) + pow(p2_y - pos_y, 2))) > 0.5:
            vBase = np.array([p2_x - p1_x, p2_y - p1_y, 0])

            pos = self._world.getTrueRobotPose()
            pos_x = pos[0]
            pos_y = pos[1]
            ori = pos[2]
            print(ori)
            e = geometry.normalToLine((pos_x, pos_y), (p1, p2))
            # print(vBase)
            #
            vT = np.array([p2_x - p1_x, p2_y - p1_y])
            print(vT)
            vT_betrag = sqrt(pow(vT[0], 2) + pow(vT[1], 2))
            vT_norm = vT / vT_betrag
            print(vT_norm)
            e_plus_r = [e[0] + vT_norm[0], e[1] + vT_norm[1]]
            print(e_plus_r)

            # print(vT)
            #
            # vDistance = np.cross(vBase, vT)
            # print(vDistance)
            # vDistanceAbs = sqrt(pow(vDistance[0], 2) + pow(vDistance[1], 2) + pow(vDistance[2], 2))
            # vBaseAbs = sqrt(pow(vBase[0], 2) + pow(vBase[1], 2) + pow(vBase[2], 2))
            #
            # e = vDistanceAbs/vBaseAbs

            theta_star = np.arctan2(e_plus_r[1], e_plus_r[0])
            delta_theta = (theta_star - ori + pi) % (2 * pi) - pi  # DAS DA, DESWEGEN FUNKTIONIERT ES

            self.move([0.5, delta_theta])
        # self.followLine(p1, p2)
        print(e)
        print(ori)
        print(theta_star)
        print(delta_theta)

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
        delta_theta = (theta - ori + pi) % (2 * pi) - pi

        print(delta_theta)
        K_w = 0.1

        # w = min(self._maxOmega, K_w * delta_theta)
        w = delta_theta
        self.move([v, w])
        self.gotoGlobal(v, p, tol)

    def followPolyline(self, v, poly):
        for point in poly:
            self.gotoGlobal(v, point, 0.5)

    def wander(self, v):
        while True:
            if self.isWall():
                break
            directions = self.getSensorDirections()
            sense = self.sense()
            min = inf
            for i in range(0, len(sense)):
                if sense[i] != None:
                    if min > sense[i]:
                        min = sense[i]
                        direction = directions[i]

            if min < 0.6 and -0.5 < direction < 0.5:
                if direction > 0:
                    rotation = -1
                else:
                    rotation = 1

                if min < 0.35:
                    self.move([v / 10, pi / 2 * rotation])
                else:
                    self.move([v / 10, pi / 4 * rotation])
            else:
                self.move([v, 0])

    def followWall(self, v, d):
        self.wander(v)

        while True:
            sense = self.sense()
            a = self.findWall(d)

            if len(a) > 0:
                end = a[0][1]
                end[1] = end[1] + d
                if sense[13] != None and sense[13] < d * 1.3:
                    self.curveDrive(v, d/3, 90)
                else:
                    direction = np.arctan2(end[1], end[0])
                    self.move([v, direction])

            else:
                self.curveDrive(v, d, -90)

    def isWall(self):
        return len(self.findWall(0.7)) > 0


    def findWall(self, d):
        sensor = self.sense()[3:12]

        for i in range(0, len(sensor)):
            if sensor[i] != None and sensor[i] > d + 0.3:
                sensor[i] = None

        directions = self.getSensorDirections()
        a = sensorUtilities.extractSegmentsFromSensorData(sensor, directions[3:12])
        return a
