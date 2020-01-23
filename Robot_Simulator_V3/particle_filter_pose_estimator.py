from math import *
import numpy as np
from random import randrange, random, uniform, gauss

from Robot_Simulator_V3 import sensorUtilities
from Robot_Simulator_V3.RobotMovement import RobotMovement


class ParticleFilterPoseEstimator:
    def __init__(self, robot: RobotMovement):
        self.robot = robot
        self.particles = []
        self._particles_weighted = []

    def initialize(self, poseFrom, poseTo, n=200):
        from_x, from_y, from_theta = poseFrom
        to_x, to_y, to_theta = poseTo
        self.particles = []
        for number in range(0, n):
            pos_x = uniform(from_x, to_x)
            pos_y = uniform(from_y, to_y)
            pos_theta = uniform(from_theta, to_theta)
            self.particles.append((pos_x, pos_y, pos_theta, 1))

    def integrateMovement(self, motion):
        for index, particle in enumerate(self.particles):
            self.particles[index] = self.move(motion, particle)

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
        v_noisy = v + gauss(0.0, sqrt(sigma_v_2))

        # Add noise to omega:
        sigma_omega_tr_2 = (self.robot._k_theta / self.robot._T) * abs(omega)  # turning rate noise
        sigma_omega_drift_2 = (self.robot._k_drift / self.robot._T) * abs(v)  # drift noise
        omega_noisy = omega + gauss(0.0, sqrt(sigma_omega_tr_2))
        omega_noisy += gauss(0.0, sqrt(sigma_omega_drift_2))

        # Move robot in the world (with noise):
        d_noisy = v_noisy * self.robot._T
        dTheta_noisy = omega_noisy * self.robot._T

        x, y, theta, _ = particle
        dx = d_noisy * cos(theta + 0.5 * dTheta_noisy)
        dy = d_noisy * sin(theta + 0.5 * dTheta_noisy)
        return x + dx, y + dy, theta + dTheta_noisy

    def integrateMeasurement(self, dist_list, alpha_list, distantMap):
        self._particles_weighted = []
        for particle in self.particles:

            p_w = 1
            coords = sensorUtilities.transformPolarCoordL2G(dist_list, alpha_list, particle[:3])
            for coord in coords:
                dist = distantMap.getValue(coord[0], coord[1])
                if dist is None:
                    dist = 100
                p_w = p_w * self.normal(0, 0.4, dist)
            self._particles_weighted.append((particle[0], particle[1], particle[2], p_w))

        M = len(self.particles)
        W = np.sum(self._particles_weighted, axis=0)[3]
        d = W / M
        r = uniform(0, d)

        self.particles = []

        for i in range(0, M):
            value = r + i * d
            weight = 0
            for particle in self._particles_weighted:
                weight += particle[3]
                if weight >= value:
                    self.particles.append((particle[0], particle[1], particle[2], particle[3]))
                    break

    def normal(self, my, sigma, x):
        return (1 / (np.sqrt(2 * pi * sigma ** 2))) * e ** -((x - my) ** 2 / 2 * sigma ** 2)

    def getPose(self):
        avg_x = 0
        avg_y = 0
        avg_theta = 0
        tot_p = 0
        for particle in self.particles:
            x, y, theta, p = particle
            avg_x += x * p
            avg_y += y * p
            avg_theta += theta * p
            tot_p += p
        avg_x = avg_x / tot_p
        avg_y = avg_y / tot_p
        avg_theta = avg_theta / tot_p
        return avg_x, avg_y, avg_theta

    def getCovariance(self):
        x_list = [x[0] for x in self.particles]
        y_list = [y[1] for y in self.particles]
        theta_list = [theta[1] for theta in self.particles]
        x_y_cov = np.cov(x_list, y_list)

        sum_theta = 0
        count = 0

        theta_list2 = []
        for theta in theta_list:
            while theta > pi:
                theta = theta - (2 * pi)

            while theta < -pi:
                theta = theta + (2 * pi)
            theta_list2.append(theta)

            sum_theta += theta
            count += 1

        avg_theta = sum_theta / count

        sigma_2 = 0
        for theta in theta_list2:
            sigma_2 += (theta-avg_theta)**2
        return (x_y_cov, sigma_2)