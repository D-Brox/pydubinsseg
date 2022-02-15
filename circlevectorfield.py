#!/usr/bin/env python

from math import atan, cos,sin,sqrt,pi

import numpy as np


def golden_section_search(f, a, b, tol=1e-5):
    gr = (sqrt(5)+1)/2
    c = b - (b - a) / gr
    d = a + (b - a) / gr
    while abs(b-a) > tol:
        if f(c) < f(d):
            b = d
        else:
            a = c
        c = b - (b - a) / gr
        d = a + (b - a) / gr
    return (b+a)/2


class CircleVectorField():

    def __init__(self, r, cx, cy, dir = 1.0, kG = 8.0, vr = 1.0):
        self.__r = lambda s: np.array([r*cos(s)+cx, r*sin(s)+cy])
        self.__drds = lambda s: np.array([-r*sin(s), r*cos(s)])
        self.__direction = dir/abs(dir)
        self.__kG = kG
        self.__vr = vr
        self.__norm_D_function = lambda p,s: np.linalg.norm( p - self.__r(s) )

    def compute_field(self,pose2D):
        p = np.array([pose2D[0], pose2D[1]])
        eta = self.__vr
        s_star = golden_section_search(lambda s: self.__norm_D_function(p,s), -pi, pi)
        r_star = self.__r(s_star)
        D = p - r_star
        T = self.__direction*self.__drds(s_star)
        G = (2/pi) * atan(self.__kG*np.linalg.norm(D))
        H = sqrt( max(1 - G**2,0) )
        F = -eta*G*D/(np.linalg.norm(D)+1e-6) + eta*H*T/(np.linalg.norm(T)+1e-6)
        return F,D,T,G,H

    def redefine(self, r, cx, cy, dir = None, kG = None, vr = None):
        self.__r = lambda s: np.array([r*cos(s)+cx, r*sin(s)+cy])
        self.__drds = lambda s: np.array([-r*sin(s), r*cos(s)])
        if dir is not None:
            self.__direction = dir/abs(dir)
        if kG is not None:
            self.__kG = kG
        if vr is not None:
            self.__vr = vr
        self.__norm_D_function = lambda p,s: np.linalg.norm( p - self.__r(s) )
