#!/usr/bin/env python
from math import pi
import math
import numpy as np

from numpy.lib.scimath import sqrt

from pydubinsseg.dubinrobot import DubinRobot
from pydubinsseg.robotmemory import RobotMemory
from pydubinsseg.circlevectorfield import CircleVectorField
from pydubinsseg import movement_will, state

def ang(p):
    return np.arctan2(p[1],p[0])

def targ_diff(pa,pb,targ):
    a = ang(pa)
    b = ang(pb)
    diff =  (2*pi - (b-a)%(2*pi)) if targ % 2 else (b-a)%(2*pi)
    # print(targ,pa,pb, a*180/pi,b*180/pi,diff*180/pi)
    return diff

def ang_diff(pa,pb):
    a = ang(pa)
    b = ang(pb)
    return min(2*pi - (b-a)%(2*pi), (b-a)%(2*pi))

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
        self.__desired_circle = 0
        self.__current_circle = 0
        self.__vector_field = CircleVectorField(1,0,0)
        self.update_memory_about_itself()

        # self.mov_radius = 1
        # self.curve_vector_field = VectorField()
        # self.transition_vector_field = VectorField()
        # self.transition_x = 0
        # self.transition_y = 0
        # self.transition_dir = 0

    def calculate_initial_conditions(self):
        while self.__robot.get_pose2D() == [0.0,0.0,0.0]:
            pass
        p = self.__robot.get_pose2D()
        self.__time_curve = self.__time
        self.__theta_curve = p[2]
        self.__current_circle = max(round(sqrt(p[0]**2 + p[1]**2)/self.__params['d']), 1)
        r = self.__current_circle*self.__params['d']
        dir = ((-1)**(self.__current_circle))
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
            'curve_index': self.__current_circle,
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
        condition_time = self.__time - self.__time_curve > time_per_curve*self.__current_circle
        if condition_theta and condition_time:
            self.__set_lap(True)

    def __set_lap(self, lap):
        self.__lap = lap
        self.__time_curve = self.__time
        self.__theta_curve = self.__robot.get_pose2D()[2]

    def calculate_wills(self):
        if not len(self.__memory.get_memory_about_others()) and not self.__lap:
            # print(len(self.__memory.get_memory_about_others()))
            return
        i_data = self.__memory.get_memory_about_itself()

        inward = False
        outward = False

        k_data_same_group_inside = []
        k_other_group_same_circle = []

        for j_data in self.__memory.get_memory_about_neighbours(self.__params["c"]):
            #A1: l13-l15
            if j_data['group'] != i_data['group'] and j_data['curve_index'] == (i_data['curve_index'] - 1):
                self.__set_lap(False)

        for k_data in self.__memory.get_memory_about_others():
            # if not k_data["state"]:
                # continue
            if k_data['group'] != i_data['group']:
                if k_data['curve_index'] == i_data['curve_index']:
                    k_other_group_same_circle.append(k_data)
            elif k_data["state"] and  k_data['curve_index'] < i_data['curve_index']:
                    k_data_same_group_inside.append(k_data)
            if k_data["state"] and k_data['will'] == movement_will['outward'] and k_data['curve_index'] == (i_data['curve_index'] - 1):
                #A1: l21-l22
                # print(self.__number,"at",self.__current_circle,"make room for",k_data["number"])
                outward = True

        k_same_group_alone_inside = False
        if k_data_same_group_inside:
            k_same_group_alone_inside = True
            for k_data in k_data_same_group_inside:
                for l_data in self.__memory.get_memory_about_others():
                    if not l_data["state"]:
                        continue
                    if k_data['curve_index'] != l_data['curve_index']:
                        continue
                    if k_data['group'] == l_data['group']:
                        continue
                    break
                else: # only executed if the inner loop did NOT break
                    continue
                k_same_group_alone_inside = False
                break

        #A1: l16-l17
        if k_same_group_alone_inside:
            inward = True
            # print(self.__number,"at",self.__current_circle,"same inward alone")
        #A1: l18-l20
        if k_other_group_same_circle and not inward:
            outward  = False
            for k_data in k_other_group_same_circle:
                if k_data["state"] and (k_data['time_curve'] < i_data['time_curve'] or (k_data['time_curve'] == i_data['time_curve'] and (k_data['pose2D'][2] < i_data['pose2D'][2]))):
                    outward |= True
                    # print(self.__number,"at",self.__current_circle,"other same", "lose to", k_data['number'],outward)
                    break
                # else:
                    # print(self.__number,"at",self.__current_circle,"other same", "win")

        #A1: l23-l24
        if self.__lap and i_data['curve_index'] > 1:
            inward = True
            print(self.__number,"at",self.__current_circle,"lap and space")

        if outward:
            self.__will = movement_will['outward']
        elif inward:
            self.__will = movement_will['inward']
        else:
            self.__will = movement_will['none']

        if inward or outward:
            self.prevent_collision(self.__memory.get_memory_about_itself(),outward,inward)

    # def check_tunnel(self, i_data, targ):
    #     curr = i_data['curve_index']

    #     d = (targ-0.5)*self.__params['d']
    #     r1 = targ*self.__params['d'] -  self.__params['Rb']
    #     r2 = self.__params['d']/2
    #     l = (r2**2 - r1**2 + d**2)/(2*d)
    #     Sr = (pi-np.arccos(l/r2))*r2

    #     Ti_Pgoal = pi*r2
    #     Ti_Pi = pi*r2 - 2*np.arccos(self.__params['Rb']/r2)*r2

    #     d = (targ+curr)/2*self.__params['d']
    #     r1 = targ*self.__params['d']
    #     r2 = self.__params['d']/2 + self.__params['Rb']
    #     l = (r1**2 - r2**2 + d**2)/(2*d)
    #     T_Pgoal_Pj = (np.arccos(l/r1))*r1

    #     return Sr,Ti_Pgoal,Ti_Pi,T_Pgoal_Pj

    def check_tunnel(self, i_data, targ):
        curr = i_data['curve_index']

        d = (targ-0.5)*self.__params['d']
        r1 = targ*self.__params['d'] -  self.__params['Rb']*2
        r2 = self.__params['d']/2
        l = (r2**2 - r1**2 + d**2)/(2*d)
        Sr = (pi-np.arccos(l/r2))*r2
        dpi = pi*r2


        d = (targ+curr)/2*self.__params['d']
        r1 = targ*self.__params['d']
        r2 = self.__params['d']/2 + self.__params['Rb']*2
        l = (r1**2 - r2**2 + d**2)/(2*d)

        ang_Sr = Sr/r1
        ang_dpi = dpi/r1
        ang_Pi = 2*np.arcsin(self.__params['Rb']/r2)*r2/r1
        ang_Pj = np.arccos(l/r1)
        # print(ang_Pi*180/pi,ang_Pj*180/pi,ang_dpi*180/pi)
        return ang_Sr,ang_Pi+ang_dpi,ang_Pj+ang_dpi

    def prevent_collision(self,i_data,outward,inward,in_a_curve_epsilon_percentage = 0.1):
        tunnel_in = []
        tunnel_out = []
        preferencial = True
        # print(i_data["number"],i_data["curve_index"],"try","inward" if inward else "","outward" if outward else "")

        for j_data in self.__memory.get_memory_about_neighbours(self.__params["c"]):
            if not j_data['state']:
                if outward:
                    self.__will = movement_will['outward']
                    return
                if inward:
                    self.__will = movement_will['inward']
                    return

            #A2: l3-l5
            if inward:
                if j_data["state"] and i_data['curve_index'] == j_data['curve_index'] + 1:
                    tunnel_in.append(j_data)

            #A2: l6-l8
            if outward:
                if j_data["state"] and i_data['curve_index'] == j_data['curve_index'] - 1:
                    tunnel_out.append(j_data)

            if j_data['state'] and j_data['will'] != movement_will['none'] and ang(j_data['pose2D']) <= ang(i_data['pose2D']):
                # print("\t",i_data["number"],"not preferencial", j_data["number"] , "is first")
                preferencial = False

        if inward:
            targ = i_data['curve_index']-1
            ang_Sr,ang_Pi_dpi, ang_Pj_dpi = self.check_tunnel(i_data,targ)
            for j_data in tunnel_in:
                if j_data["group"] == i_data["group"]:
                    ang_Sr_l=2*ang_Sr
                else:
                    ang_Sr_l = ang_Sr

                ang_before = ang_Pi_dpi + ang_Sr_l
                ang_after = ang_Pj_dpi + ang_Sr_l
                ang_diff = math.remainder(targ_diff(i_data['pose2D'],j_data['pose2D'],targ),2*pi)
                # print(ang(i_data["pose2D"])*180/pi,i_data["curve_index"],ang(j_data["pose2D"])*180/pi,j_data["curve_index"],"\t",-ang_before*180/pi,ang_diff*180/pi,ang_after*180/pi,"\t", ang_Pi_dpi*180/pi,ang_Sr_l*180/pi, ang_Pj_dpi*180/pi)
                # print(i_data["pose2D"][:2],j_data["pose2D"][:2])
                if -ang_before < ang_diff < ang_after:
                    inward = False
                    break
        if outward:
            targ = i_data['curve_index']-1
            ang_Sr,ang_Pi_dpi, ang_Pj_dpi = self.check_tunnel(i_data,targ)
            for j_data in tunnel_out:
                if j_data["group"] == i_data["group"]:
                    ang_Sr_l=2*ang_Sr
                else:
                    ang_Sr_l = ang_Sr

                ang_before = ang_Pi_dpi + ang_Sr_l
                ang_after = ang_Pj_dpi + ang_Sr_l
                ang_diff = math.remainder(targ_diff(i_data['pose2D'],j_data['pose2D'],targ),2*pi)
                # print(ang(i_data["pose2D"])*180/pi,i_data["curve_index"],ang(j_data["pose2D"])*180/pi,j_data["curve_index"],"\t",-ang_before*180/pi,ang_diff*180/pi,ang_after*180/pi,"\t", ang_Pi_dpi*180/pi,ang_Sr_l*180/pi, ang_Pj_dpi*180/pi)
                # print(i_data["pose2D"][:2],j_data["pose2D"][:2])
                if -ang_before < ang_diff < ang_after:
                    outward = False
                    break

        if outward and preferencial:
            # print("\t\t",i_data["number"],i_data["curve_index"],"outward")

            self.__will = movement_will['outward']
            self.evaluate_will_field()
        elif inward and preferencial:
            self.__will = movement_will['inward']
            # print("\t\t",i_data["number"],i_data["curve_index"],"inward")
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
            self.__desired_circle = max(self.__current_circle+sig,1)
            dir = ((-1)**(self.__desired_circle)) if sig == 1 else None
            self.__vector_field.redefine(r, cx, cy, dir = dir)

    def calculate_input_signals(self):
        [F,D,T,G,H] = self.__vector_field.compute_field(self.__robot.get_pose2D())
        return self.__robot.calculate_lower_control(F)

    def check_arrival(self, tol = 0.5):
        r = self.__desired_circle*self.__params['d']
        [F,D,T,G,H] = CircleVectorField(r,0,0).compute_field(self.__robot.get_pose2D())
        if np.linalg.norm(D) <= tol:
            self.__set_lap(False)
            self.set_state(state['in circle'])
            self.__will = movement_will['none']
            self.__memory.reset()
            self.__current_circle = self.__desired_circle
            dir = ((-1)**(self.__current_circle))
            self.__vector_field.redefine(r,0,0,dir=dir)
