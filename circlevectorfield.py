#!/usr/bin/env python

import numpy as np
from numpy import cos,sin,pi

from pydubinsseg.vector_utils import ang_vec, vec_norm, ortogonal_vec

class CircleVectorField():

    def __init__(self, r, cx, cy, dir = 1.0, vr = 1.0):
        self.__c_r = [np.array([cx,cy]),r]
        self.__r = lambda s: np.array([r*cos(s)+cx, r*sin(s)+cy])
        self.__direction = dir/abs(dir)
        self.__vr = vr

    def compute_field(self,pose2D):
        p = np.array([pose2D[0], pose2D[1]])
        eta = self.__vr

        c,r = self.__c_r
        p_c,p_c_norm = vec_norm(p,c)
        s_star = ang_vec(p_c)
        r_star = self.__r(s_star)

        D, dnorm = vec_norm(p,r_star)
        if dnorm != 0:
            N, norm = vec_norm(D,0.1*D/dnorm)
            if norm == 0:
                norm = 1e-3
            T = self.__direction*ortogonal_vec(p_c)/p_c_norm
            G = - norm/np.sqrt(1 + norm**2)
            H = 1/np.sqrt(1+norm**2)
        else:
            N = D
            T = self.__direction*ortogonal_vec(p_c)/p_c_norm
            G = 0
            H = 1

        F =  eta*(G*N+H*T)
        delta = p_c_norm/r
        return F,D,T,G,H,delta

    def redefine(self, r, cx, cy, dir = None, kG = None, vr = None):
        self.__c_r = [np.array([cx,cy]),r]
        self.__r = lambda s: np.array([r*cos(s)+cx, r*sin(s)+cy])
        if dir is not None:
            self.__direction = dir/abs(dir)
        if vr is not None:
            self.__vr = vr
