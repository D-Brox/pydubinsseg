#!/usr/bin/env python
from math import pi
import numpy as np

from numpy.lib.scimath import sqrt

from dubinrobot import DubinRobot
from robotmemory import RobotMemory
from pydubinsseg import movement_will, state
from pydubinsseg.circlevectorfield import CircleVectorField


class SegregationControl():

    def __init__(self, number, group, state, params):
        self.__number = number
        self.__group = group
        self.__params = params
        self.__state = state
        self.__will = movement_will['none']
        self.__lap = False
        self.__robot = DubinRobot(1.0)
        self.__memory = RobotMemory()

        self.__time = 0
        self.__time_curve = 0
        self.__theta_curve = 0
        self.__closest_circle = 0
        self.__vector_field = CircleVectorField(1,0,0)

        # self.mov_radius = 1
        # self.curve_vector_field = VectorField()
        # self.transition_vector_field = VectorField()
        # self.transition_x = 0
        # self.transition_y = 0
        # self.transition_dir = 0

    def calculate_initial_conditions(self):
        p = self.__robot.get_pose2D()
        self.__time_curve = self.__time
        self.__theta_curve = p[2]
        self.__closest_circle = max(round(sqrt(p[0]**2 + p[1]**2)/self.__params['d']), 1)
        r = self.__closest_circle*self.__params['d']
        dir = ((-1)**(self.__closest_circle))
        self.__vector_field.redefine(r,0,0,dir=dir)

    def get_state(self):
        return self.__state
        
    def set_state(self, state):
        self.__state = state

    def set_time(self, time):
        self.__time = time

    def set_pose2D(self, pose2D):
        self.__robot.set_pose2D(pose2D)

    def get_pose2D(self):
        return self.__robot.get_pose2D()

    def set_params(self, params):
        self.__params = params
        if 'ref_vel' in self.__params:
            self.__robot.set_constant_linear_velocity(self.__params['ref_vel'])

    def update_memory_about_itself(self):
        data = {
            'curve_index': self.__closest_circle,
            'time_curve': self.__time_curve,
            'time': self.__time,
            'pose2D': self.__robot.get_pose2D(),
            'will': self.__will
        }
        self.__memory.update_memory_about_itself(data)

    def send_memory(self):
        return self.__memory.get_memory()

    def recieve_memory(self, other_memory):
        self.__memory.compare_and_update(other_memory)

    def calculate_lap(self):
        error_limit = 0.01
        time_per_curve = 60
        theta_diff = abs(self.__theta_curve - self.__robot.get_pose2D()[2])
        condition_theta = theta_diff <= error_limit or theta_diff >= 2*pi-error_limit
        condition_time = self.__time - self.__time_curve > time_per_curve*self.__closest_circle 
        if condition_theta and condition_time:
            self.__set_lap(True)
    
    def __set_lap(self, lap):
        self.__lap = lap
        self.__time_curve = self.__time
        self.__theta_curve = self.__robot.get_pose2D()[2]

    def calculate_wills(self):
        inward = False
        outward = False
        i_data = self.__memory.get_memory_about_itself()
        for j_data in self.__memory.get_memory_about_others():
            j_is_inward = j_data['curve_index'] < i_data['curve_index']
            j_is_immediate_inward = j_data['curve_index'] == i_data['curve_index'] - 1
            j_is_same_curve = j_data['curve_index'] == i_data['curve_index']
            j_is_same_group = j_data['group'] == i_data['group']
            j_wants_outward = j_data['will'] == movement_will['outward']
            j_arrived_first = j_data['time_curve'] < i_data['time_curve']
            j_tiebreaker = (j_data['time_curve'] == i_data['time_curve']) and (j_data['pose2D'][2] < i_data['pose2D'][2])
            j_group_is_alone = True
            for k_data in self.__memory.get_memory_about_others():
                if k_data['number'] == j_data['number']:
                    continue
                k_is_same_group_j = k_data['group'] == j_data['group']
                k_is_same_curve_j = k_data['curve_index'] == j_data['curve_index']
                if k_is_same_curve_j and not k_is_same_group_j:
                    j_group_is_alone = False
                    break
            if j_is_immediate_inward and not j_is_same_group:
                self.__set_lap(False)
            if j_is_same_group and j_group_is_alone and j_is_inward:
                inward = True
            if j_is_same_curve and not j_is_same_group and not inward:
                if j_arrived_first or j_tiebreaker:
                    outward  = True
            if j_is_immediate_inward and j_wants_outward:
                outward = True

        if self.__lap and i_data['curve_index'] > 1:
            inward = True

        if outward:
            self.__will = movement_will['outward']
        elif inward:
            self.__will = movement_will['inward']
        else:
            self.__will = movement_will['none']

    def prevent_collision(self,in_a_curve_epsilon_percentage = 0.1):
        pass

    def evaluate_wills(self):
        if self.__will == movement_will['outward']:
            self.set_state(state['transition'])
            self.__set_lap(False)
            p = self.__robot.get_pose2D()
            r = self.__params['d']/2
            cx = p[0]*(1 + self.__params['d']/(2*sqrt(p[0]**2 + p[1]**2)))
            cy = p[1]*(1 + self.__params['d']/(2*sqrt(p[0]**2 + p[1]**2)))
            self.__vector_field.redefine(r, cx, cy)
            self.__closest_circle = self.__closest_circle + 1
        elif self.__will == movement_will['inward']:
            self.set_state(state['transition'])
            self.__set_lap(False)
            p = self.__robot.get_pose2D()
            r = self.__params['d']/2
            cx = p[0]*(1 - self.__params['d']/(2*sqrt(p[0]**2 + p[1]**2)))
            cy = p[1]*(1 - self.__params['d']/(2*sqrt(p[0]**2 + p[1]**2)))
            self.__vector_field.redefine(r, cx, cy)
            self.__closest_circle = max(self.__closest_circle - 1, 1)

    def calculate_input_signals(self):
        [F,D,T,G,H] = self.__vector_field.compute_field(self.__robot.get_pose2D())
        return self.__robot.calculate_lower_control(F)

    def check_arrival(self, tol = 0.5):
        r = self.__closest_circle*self.__params['d']
        [F,D,T,G,H] = CircleVectorField(r,0,0).compute_field(self.__robot.get_pose2D())
        if np.linalg.norm(D) <= tol:
            self.__set_lap(False)
            self.set_state(state['in circle'])
            self.__will = movement_will['none']
            self.__memory.reset()
            dir = ((-1)**(self.__closest_circle))
            self.__vector_field.redefine(r,0,0,dir=dir)
