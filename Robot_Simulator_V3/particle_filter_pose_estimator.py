from math import *
import numpy as np
from random import randrange, random, uniform

from Robot_Simulator_V3.RobotMovement import RobotMovement


class ParticleFilterPoseEstimator:

    def __init__(self, robot: RobotMovement):
        self.robot = robot
        self._particles = []
        self._particles_weighted = []

    def initialize(self, poseFrom, poseTo, n=200):
        from_x, from_y, from_theta = poseFrom
        to_x, to_y, to_theta = poseTo
        self._particles = []
        for number in range(0, 200):
            pos_x = uniform(from_x, to_x)
            pos_y = uniform(from_y, to_y)
            pos_theta = uniform(from_theta, to_theta)
            self._particles.append((pos_x, pos_y, pos_theta))

    def integrateMovement(self, motion):
        for index, particle in enumerate(self._particles):
            self._particles[index] = self.move(motion, particle)
            # self._particles[index].undraw()
            # p = Point(x + dx + self.r * cos(theta), y + dy + self.r * sin(theta))
            # self.lines[i] = Line(nc, p)
            # self.lines[i].draw(myWorld._win)
            # self.lines[i].setWidth(3)

    def move(self, motion, particle):
        v = motion[0]
        omega = motion[1]

        # translational and rotational speed is limited:
        if omega > self.robot._maxOmega:
            omega = self.robot._maxOmega
        if omega < -self.robot._maxOmega:
            omega = -self.robot._maxOmega
        if v > self.robot._maxSpeed:
            v = self.robot._maxSpeed
        if v < -self.robot._maxSpeed:
            v = -self.robot._maxSpeed

        # Add noise to v:
        sigma_v_2 = (self.robot._k_d / self.robot._T) * abs(v)
        v_noisy = v + random.gauss(0.0, sqrt(sigma_v_2))

        # Add noise to omega:
        sigma_omega_tr_2 = (self.robot._k_theta / self.robot._T) * abs(omega)  # turning rate noise
        sigma_omega_drift_2 = (self.robot._k_drift / self.robot._T) * abs(v)  # drift noise
        omega_noisy = omega + random.gauss(0.0, sqrt(sigma_omega_tr_2))
        omega_noisy += random.gauss(0.0, sqrt(sigma_omega_drift_2))

        # Move robot in the world (with noise):
        d_noisy = v_noisy * self.robot._T
        dTheta_noisy = omega_noisy * self.robot._T

        x, y, theta = particle
        dx = d_noisy * cos(theta + 0.5 * dTheta_noisy)
        dy = d_noisy * sin(theta + 0.5 * dTheta_noisy)
        return x + dx, y + dy, theta + dTheta_noisy

    def integrateMeasurement(self, dist_list, alpha_list, distantMap):
        self._particles_weighted = []
        for particle in self._particles:
            p_x, p_y, p_theta = particle

            p_w = 1
            for sense in zip(dist_list, alpha_list):
                d, alpha = sense
                x_z = d * np.cos(alpha + p_theta)
                y_z = d * np.sin(alpha + p_theta)

                x_z = x_z + p_x
                y_z = y_z + p_y

                dist = distantMap.getValue(x_z, y_z)
                p_w = p_w * self.normal(0, 0.4, dist)
            self._particles_weighted.append((p_x, p_y, p_theta, p_w))

    def normal(self, my, sigma, x):
        return (1 / (np.sqrt(2 * pi * sigma**2))) * e**-((x - my) ** 2 / 2 * sigma ** 2)

    def getPose(self):
        avg_x = 0
        avg_y = 0
        avg_theta = 0
        tot_p = 0
        for particle in self._particles_weighted:
            x, y, theta, p = particle
            avg_x += x
            avg_y += y
            avg_theta += theta
            tot_p += p
        avg_x = avg_x/tot_p
        avg_y = avg_y/tot_p
        avg_theta = avg_theta/tot_p
        return avg_x, avg_y, avg_theta

    def getCovariance(self):
        x_list = [x[0] for x in self._particles]
        y_list = [y[1] for y in self._particles]
        theta_list = [theta[1] for theta in self._particles]
        np.cov(x_list, y_list)

        sum_theta = 0

        for theta in theta_list:
            sin_t = sin(theta)
            sum_theta =  #Im Einheitskreis mean berechnen, im zweifelsfall noch mal googeln



