import math
import numpy as np

""" Fx = np.zeros((3, 2*4 + 3))
for i in range(3):
    Fx[i,i] = 1

print(Fx)
transposed = list(zip(*Fx))

for row in transposed:
    print(row)    """

def jacobian(v,w,t,theta,Fx,n):
    Gx = np.zeros((3,3))
    Gx[0,2] = -(v/w) * math.cos(theta) + (v/w) * math.cos(theta + w*t)
    Gx[1,2] = -(v/w) * math.sin(theta) + (v/w) * math.sin(theta + w*t)

    Gt = np.identity(2*n+3)
    Gt = Gt + list(zip(*Fx)) @ Gx @ Fx 

    return Gt

def covariance(Gt,cov,Fx,Rt,n):
    p_cov = np.zeros((2*n+3,2*n+3))
    p_cov = Gt @ cov @ list(zip(*Gt)) + list(zip(*Fx)) @ Rt @ Fx
    return p_cov
