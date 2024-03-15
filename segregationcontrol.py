#!/usr/bin/env python
from math import pi
import numpy as np

from numpy.lib.scimath import sqrt

from pydubinsseg.dubinrobot import DubinRobot
from pydubinsseg.robotmemory import RobotMemory
from pydubinsseg.circlevectorfield import CircleVectorField
from pydubinsseg import movement_will, state


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

        self.update_memory_about_itself()

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

    def get_group(self):
        return self.__group
        
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
            'group': self.__group,
            'number': self.__number,
            'curve_index': self.__closest_circle,
            'time_curve': self.__time_curve,
            'time': self.__time,
            'pose2D': self.__robot.get_pose2D(),
            'will': self.__will,
            'state': self.__state
        }
        self.__memory.update_memory_about_itself(data)

    def send_memory(self):
        return self.__memory.get_memory()

    def recieve_memory(self, other_memory):
        self.__memory.compare_and_update(other_memory)


    #HACK: think of a better way to do this
    def calculate_lap(self):
        error_limit = 0.01
        time_per_curve = 60 # wishful thinking
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
        i_data = self.__memory.get_memory_about_itself()

        inward = False
        outward = False

        k_data_same_group = []
        k_other_group_same_circle = []
        make_room = False

        for j_data in self.__memory.get_memory_about_others():
            if j_data['group'] == i_data['group']:
                k_data_same_group.append(j_data)
            else:
                j_is_immediate_inward = j_data['curve_index'] == (i_data['curve_index'] - 1)
                if j_is_immediate_inward:
                    self.__set_lap(False)
                if j_data['curve_index'] == i_data['curve_index']:
                    k_other_group_same_circle.append(j_data)
                if j_data['will'] == movement_will['outward'] and j_is_immediate_inward:
                    make_room |= True

        k_same_group_alone = False
        if k_data_same_group:
            k_same_group_alone = True
            for k_data in k_data_same_group:
                for l_data in self.__memory.get_memory_about_others():
                    if k_data['curve_index'] != l_data['curve_index']:
                        continue
                    if k_data['group'] == l_data['group']:
                        continue
                    break
                else: # only executed if the inner loop did NOT break
                    if k_data['curve_index'] < i_data['curve_index']:
                        continue
                k_same_group_alone = False
                break

        if k_same_group_alone:
            inward = True
            print(self.__number,"at",self.__closest_circle,"same inward alone")
        if k_other_group_same_circle and not inward:
            outward  = False
            for k_data in k_other_group_same_circle:
                if k_data['time_curve'] < i_data['time_curve'] or (k_data['time_curve'] == i_data['time_curve'] and (k_data['pose2D'][2] < i_data['pose2D'][2])):
                    print(self.__number,"at",self.__closest_circle,"other same", "lose to", k_data['number'])
                    outward |= True
                else:
                    print(self.__number,"at",self.__closest_circle,"other same", "win")
        if make_room:
            outward = True
            print(self.__number,"at",self.__closest_circle,"make room")

        if self.__lap and i_data['curve_index'] > 1:
            inward = True
            print(self.__number,"at",self.__closest_circle,"lap and space")

        self.prevent_collision(i_data,outward,inward)

    def check_tunnel(self, i_data, j_data):
        curr = i_data['curve_index']
        ang_i = i_data['pose2D'][2]
        targ = j_data['curve_index']
        ang_j = j_data['pose2D'][2]
        ang_diff = lambda a,b: min(2*pi - (a-b)%2*pi, (a-b)%2*pi)

        d = (targ-0.5)*self.__params['d']
        r1 = targ*self.__params['d'] -  self.__params['Rb']
        r2 = self.__params['d']/2
        l = (r2**2 - r1**2 + d**2)/(2*d)
        Sr = (pi-np.arccos(l/r2))*r2
        if i_data['group'] != j_data['group']:
            Sr*=2

        dir_i = (-1)**curr
        dir_j = (-1)**targ

        # print(curr,dir_i,ang_i,targ,dir_j,ang_j)

        Ti_Pgoal = pi*r2
        Ti_Pi = pi*r2 - (r2-(2*self.__params['Rb'])**2/r2)

        Tj_Pgoal_plus = ang_diff(ang_i,ang_j)*targ*self.__params['d'] + Sr

        d = (targ+curr)*self.__params['d']/2
        r1 = targ*self.__params['d']
        r2 = self.__params['d']/2 + self.__params['Rb']
        l = (r1**2 - r2**2 + d**2)/(2*d)
        Pj = (np.arccos(l/r1))*r1
        Tj_Pj_minus = Pj - Sr

        # return True

        if Ti_Pi > Tj_Pgoal_plus or Tj_Pj_minus > Ti_Pgoal:
            # print(i_data["number"], "safe")
            return True
        else:
            # print("close",ang_i,ang_j,Ti_Pi,Tj_Pgoal_plus,dir_j,Tj_Pj_minus,Ti_Pgoal)
            return False

    def prevent_collision(self,i_data,outward,inward,in_a_curve_epsilon_percentage = 0.1):
        d = self.__params['d']
        j_in_a_circle = []
        no_preferential = True
        for j_data in self.__memory.get_recent_memory_about_others():
            pj = j_data['pose2D']
            j_in_a_circle.append(j_data['state'])
            if inward and i_data['curve_index'] == j_data['curve_index'] + 1:
                # print(i_data['number']," inward", j_data['number'],"at", j_data["curve_index"] )
                if not self.check_tunnel(i_data,j_data):
                    inward = False
            if outward and i_data['curve_index'] == j_data['curve_index'] - 1:
                # print(i_data['number'],"outward", j_data['number'],"at", j_data["curve_index"] )
                if not self.check_tunnel(i_data,j_data):
                    outward = False
            if j_data['will'] != movement_will['none'] and pj[2] <= i_data['pose2D'][2]:
                no_preferential = False

        if (not outward) and inward and all(j_in_a_circle) and no_preferential:
            self.__will = movement_will['inward']
            self.evaluate_will_field()
        elif outward and all(j_in_a_circle) and no_preferential:
            self.__will = movement_will['outward']
            self.evaluate_will_field()
        else:
            self.__will = movement_will['none']

    def evaluate_will_field(self):
        if self.__will != movement_will['none'] and self.get_state() == state['in circle']:
            self.set_state(state['transition'])
            self.__set_lap(False)
            p = self.__robot.get_pose2D()
            r = self.__params['d']/2
            sig = 1 if self.__will == movement_will['outward'] else -1
            cx = p[0]*(sqrt(p[0]**2 + p[1]**2)+sig*r)/sqrt(p[0]**2 + p[1]**2)
            cy = p[1]*(sqrt(p[0]**2 + p[1]**2)+sig*r)/sqrt(p[0]**2 + p[1]**2)
            self.__closest_circle = max(self.__closest_circle+sig,1)
            dir = ((-1)**(self.__closest_circle)) if sig == 1 else None
            self.__vector_field.redefine(r, cx, cy, dir = dir)

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
