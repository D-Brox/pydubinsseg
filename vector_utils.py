import numpy as np
from numpy import pi
from numpy.lib.scimath import sqrt

def ang_vec(p):
    return np.arctan2(p[1],p[0])

def targ_diff(pa,pb,targ):
    a = ang_vec(pa)
    b = ang_vec(pb)
    diff =  (b-a)%(2*pi) if targ % 2 else (a-b)%(2*pi)
    if diff > pi:
        diff -= pi
    if diff < -pi:
        diff += pi
    return diff

def ang_diff(a,b):
    return min(2*pi - (b-a)%(2*pi), (b-a)%(2*pi))

def vec_norm(p1,p2):
    vec = p1 - p2
    return vec, sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def ortogonal_vec(v):
    return np.array([-v[1],v[0]])
