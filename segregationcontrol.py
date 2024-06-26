#!/usr/bin/env python
import numpy as np
from numpy import pi
from numpy.lib.scimath import sqrt

from pydubinsseg.dubinrobot import DubinRobot
from pydubinsseg.robotmemory import RobotMemory
from pydubinsseg.circlevectorfield import CircleVectorField
from pydubinsseg.vector_utils import ang_vec, ang_vec_diff, targ_diff
from pydubinsseg import movement_will, state

class SegregationControl():

    def __init__(self, number, group, state, params):
        self.__number = number
        self.__group = group
        self.__params = params
        self.__state = state
        self.__will = movement_will["none"]
        self.__lap = False
        self.__robot = DubinRobot(1.0)
        self.__memory = RobotMemory()

        self.__time = 0
        self.__start_time = 10
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
        self.__current_circle = max(round(sqrt(p[0]**2 + p[1]**2)/self.__params["d"]), 1)
        r = self.__current_circle*self.__params["d"]
        dir = ((-1)**(self.__current_circle))
        self.__vector_field.redefine(r,0,0,dir=dir)
        self.update_memory_about_itself()

    def get_state(self):
        return self.__state

    def get_group(self):
        return self.__group

    def set_neighbors(self,neighbors):
        self.__memory.set_neighbors(neighbors)

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
        if "ref_vel" in self.__params:
            self.__robot.set_constant_linear_velocity(self.__params["ref_vel"])

    def update_memory_about_itself(self):
        data = {
            "group": self.__group,
            "number": self.__number,
            "curve": self.__current_circle,
            "time_curve": self.__time_curve,
            "time": self.__time,
            "pose2D": self.__robot.get_pose2D(),
            "will": self.__will,
            "state": self.__state
        }
        self.__memory.update_memory_about_itself(data)

    def send_memory(self):
        return self.__memory.get_memory()

    def recieve_memory(self, other_memory):
        self.__memory.compare_and_update(other_memory)


    #HACK: think of a better way to do this
    def calculate_lap(self):
        error_limit = 0.01
        time_per_curve = 60 # wishful thinking, depending on the PC
        theta_diff = abs(self.__theta_curve - self.__robot.get_pose2D()[2])
        condition_theta = theta_diff <= error_limit or theta_diff >= 2*pi-error_limit
        condition_time = self.__time - self.__time_curve > time_per_curve*self.__current_circle
        if condition_theta and condition_time:
            self.__set_lap(True)

    def __set_lap(self, lap):
        self.__lap = lap
        self.__time_curve = self.__time
        self.__theta_curve = self.__robot.get_pose2D()[2]
        self.update_memory_about_itself()

    def calculate_will(self):
        i_data = self.__memory.get_memory_about_itself()

        make_room = False
        inward = False
        outward = False

        neighbors_same_circle = []
        k_data_same_group_inside = []
        k_other_group_same_circle = []

        for j_data in self.__memory.get_memory_about_neighbors():
            # A1: l13-l15
            if j_data["group"] != i_data["group"] and j_data["curve"] == i_data["curve"] - 1:
                self.__set_lap(False)

            if j_data["curve"] == i_data["curve"]:
                neighbors_same_circle.append(j_data)

        for k_data in self.__memory.get_memory_about_others():
            # if not k_data["state"]:
                # continue
            if k_data["group"] != i_data["group"]:
                if k_data["curve"] == i_data["curve"]:
                    k_other_group_same_circle.append(k_data)
            elif k_data["curve"] < i_data["curve"]:
                    k_data_same_group_inside.append(k_data)
            if k_data["group"] != i_data["group"] and k_data["will"] == movement_will["outward"] and k_data["curve"] == (i_data["curve"] - 1):
                # print(self.__number,"at",self.__current_circle,"make room for",k_data["number"])
                make_room = True

        k_same_group_alone_inside = False
        if k_data_same_group_inside:
            k_same_group_alone_inside = True
            for k_data in k_data_same_group_inside:
                for l_data in self.__memory.get_memory_about_others():
                    if not l_data["state"]:
                        continue
                    if k_data["curve"] != l_data["curve"]:
                        continue
                    if k_data["group"] == l_data["group"]:
                        continue
                    break
                else: # only executed if the inner loop did NOT break
                    continue
                k_same_group_alone_inside = False
                break

        #A1: l22-l23
        if make_room:
            close = []
            ang_Sr = self.security_distance(i_data["curve"])/(i_data["curve"] * self.__params["d"])
            for j_data in neighbors_same_circle:
                if ang_vec_diff(i_data["pose2D"],j_data["pose2D"]) < 1.5 * ang_Sr:
                    close.append(j_data)
            if len(close) >= 2:
                outward = True

        #A1: l17-l16
        if k_same_group_alone_inside:
            inward = True
            # print(self.__number,"at",self.__current_circle,"same inward alone")
        #A1: l19-l21
        if k_other_group_same_circle and not inward:
            outward  = False
            for k_data in k_other_group_same_circle:
                # print(i_data["curve"],i_data["time_curve"],k_data["time_curve"])
                if k_data["state"] and (i_data["time_curve"] > k_data["time_curve"] or (i_data["time_curve"] == k_data["time_curve"] and (i_data["pose2D"][2] > k_data["pose2D"][2]))):
                    outward |= True
                    # print(self.__number,"at",self.__current_circle,"other same", "lose to", k_data["number"],outward)
                    break
                # else:
                    # print(self.__number,"at",self.__current_circle,"other same", "win")

        #A1: l22-l23
        if self.__lap and i_data["curve"] > 1:
            inward = True
            # print(self.__number,"at",self.__current_circle,"lap and space")

        if outward:
            self.__will = movement_will["outward"]
        elif inward:
            self.__will = movement_will["inward"]
        else:
            self.__will = movement_will["none"]

        self.update_memory_about_itself()
        return inward,outward

    def security_distance(self,curve):
        d = (curve-0.5)*self.__params["d"]
        r1 = curve*self.__params["d"] -  self.__params["Rb"]*2
        r2 = self.__params["d"]/2
        l = (r2**2 - r1**2 + d**2)/(2*d)
        return (pi-np.arccos(l/r2))*r2

    def tunnel_angles(self, curr, targ):
        d = (targ+curr)/2*self.__params["d"]
        r1 = targ*self.__params["d"]
        r2 = self.__params["d"]/2 + self.__params["Rb"]*2
        l = (r1**2 - r2**2 + d**2)/(2*d)

        r = self.__params["d"]/2
        goal = pi*r
        ang_goal = goal/r1

        Sr = self.security_distance(targ)
        ang_Sr = Sr/r1

        ang_Pi = 2*np.arcsin(self.__params["Rb"]/r2)*r/r1
        ang_Pj = np.arccos(l/r1)
        # print(ang_Pi*180/pi,ang_Pj*180/pi,ang_dpi*180/pi)
        return ang_Sr,ang_goal,ang_Pi,ang_Pj

    def prevent_collision(self,inward,outward):
        if self.__time < self.__start_time:
            return
        i_data = self.__memory.get_memory_about_itself()
        tunnel_in = []
        tunnel_out = []
        preferencial = True
        for j_data in self.__memory.get_memory_about_neighbors():
            if abs(i_data["curve"] - j_data["curve"]) > 2:
                # print("\t\t",i_data["curve"],j_data["curve"])
                continue

            if not j_data["state"]:
                # print(i_data["curve"],"cancel state",j_data["curve"])
                preferencial = False

            #A2: l3-l5
            if inward:
                if i_data["curve"] - 1 == j_data["curve"]:
                    tunnel_in.append(j_data)

            #A2: l6-l8
            if outward:
                if i_data["curve"] + 1 == j_data["curve"]:
                    tunnel_out.append(j_data)

            if j_data['will'] != movement_will['none'] and ang_vec(j_data['pose2D']) < ang_vec(i_data['pose2D']):
                # print("\t",i_data["number"],"not preferencial", j_data["number"] , "is first")
                if j_data["curve"] == i_data["curve"]:
                    preferencial = False
                    # print(i_data["curve"],"cancel pref same")
                else:
                    if outward and j_data["curve"] > i_data["curve"]:
                        preferencial = False
                        # print(i_data["curve"],"cancel pref out")
                    if inward and j_data["curve"] < i_data["curve"]:
                        preferencial = False
                        # print(i_data["curve"],"cancel pref in")

        if inward:
            targ = i_data["curve"] - 1
            ang_Sr,ang_goal,ang_Pi, ang_Pj = self.tunnel_angles(i_data["curve"],targ)
            for j_data in tunnel_in:
                if j_data["group"] == i_data["group"] and targ != 1:
                    ang_Sr_l=2*ang_Sr
                else:
                    ang_Sr_l = ang_Sr

                ang_before = ang_goal - ang_Pi - ang_Sr_l
                ang_after = ang_goal + ang_Pj + ang_Sr_l
                ang_diff = targ_diff(i_data["pose2D"],j_data["pose2D"],targ)
                # print(i_data["pose2D"][:2],j_data["pose2D"][:2])
                # print(i_data["curve"],"inward",ang_before*180/pi,ang_diff*180/pi,ang_after*180/pi, ang_before < ang_diff < ang_after)
                if ang_before < ang_diff < ang_after:
                    inward = False
                    # print(i_data["curve"],"cancel in dist")
                    break

        if outward:
            targ = i_data["curve"] + 1
            ang_Sr,ang_goal,ang_Pi, ang_Pj = self.tunnel_angles(i_data["curve"],targ)
            for j_data in tunnel_out:
                if j_data["group"] == i_data["group"]:
                    ang_Sr_l=2*ang_Sr
                else:
                    ang_Sr_l = ang_Sr

                ang_before = ang_goal - ang_Pi - ang_Sr_l
                ang_after = ang_goal + ang_Pj + ang_Sr_l
                ang_diff = targ_diff(i_data["pose2D"],j_data["pose2D"],targ)
                # print(i_data["pose2D"][:2],j_data["pose2D"][:2])
                # print(i_data["curve"],"outward", ang_before*180/pi,ang_diff*180/pi,ang_after*180/pi, ang_before < ang_diff < ang_after)
                if ang_before < ang_diff < ang_after:
                    outward = False
                    # print(i_data["curve"],"cancel out dist")
                    break

        if outward:
            self.__will = movement_will["outward"]
            # print("\t\t",i_data["number"],i_data["curve"],"outward")
            if preferencial:
                self.evaluate_transition_field()
        elif inward:
            self.__will = movement_will["inward"]
            # print("\t\t",i_data["number"],i_data["curve"],"inward")
            if preferencial:
                self.evaluate_transition_field()
        self.update_memory_about_itself()

    def evaluate_transition_field(self):
        self.set_state(state["transition"])
        self.__set_lap(False)
        p = self.__robot.get_pose2D()
        r = self.__params["d"]/2
        sig = 1 if self.__will == movement_will["outward"] else -1
        cx = p[0]*(sqrt(p[0]**2 + p[1]**2)+sig*r)/sqrt(p[0]**2 + p[1]**2)
        cy = p[1]*(sqrt(p[0]**2 + p[1]**2)+sig*r)/sqrt(p[0]**2 + p[1]**2)
        self.__desired_circle = max(self.__current_circle+sig,1)
        dir = ((-1)**(self.__desired_circle)) if sig == 1 else None
        self.__vector_field.redefine(r, cx, cy, dir = dir)

    def calculate_input_signals(self):
        [F,_,_,_,_,delta] = self.__vector_field.compute_field(self.__robot.get_pose2D())
        return self.__robot.calculate_lower_control(F,delta)

    def check_arrival(self, tol = 0.2):
        r = self.__desired_circle*self.__params["d"]
        [_,D,_,_,_,_] = CircleVectorField(r,0,0).compute_field(self.__robot.get_pose2D())
        if np.linalg.norm(D) <= tol:
            self.__set_lap(False)
            self.set_state(state["in circle"])
            self.__will = movement_will["none"]
            self.__memory.neighbors()
            self.__current_circle = self.__desired_circle
            dir = ((-1)**(self.__current_circle))
            self.__vector_field.redefine(r,0,0,dir=dir)
        self.update_memory_about_itself()
