#!/usr/bin/env python
from math import atan2,sin,asin


class DubinRobot():

    def __init__(self, v):
        self.__x = 0
        self.__y = 0
        self.__theta = 0
        self.__constant_linear_velocity = v
        # self.__range_sensor = RangeSensor()

    def set_pose2D(self, pose2D):
        self.__x = float(pose2D[0])
        self.__y = float(pose2D[1])
        self.__theta = float(pose2D[2])

    def get_pose2D(self):
        return [self.__x, self.__y, self.__theta]

    def set_constant_linear_velocity(self, v):
        self.__constant_linear_velocity = v

    def calculate_lower_control(self, F, delta):
        Kp = 0.8
        theta_ref = atan2(F[1], F[0])
        theta_error = asin(sin(theta_ref - self.__theta))
        w = Kp*theta_error
        v = self.__constant_linear_velocity * delta
        return (v,w)
