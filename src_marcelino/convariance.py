import math
import numpy as np

""" Fx = np.zeros((3, 2*4 + 3))
for i in range(3):
    Fx[i,i] = 1

print(Fx)
transposed = list(zip(*Fx))

for row in transposed:
    print(row)    """

def jacobian(v, w, t, theta, Fx, n):
    """
    Jacobian of motion obtained from the derivative of the position plus the translation matrix

    Arguments:
    -v: current speed of the robot
    -w: current angular speed of the robot
    -t: time interval since the last update
    theta: current orientation of the robot   
    Fx: matrix Fx
    n: total number of landmarks

    Returns:
    -returns the jacobian matrix of motion Gt
    """

    Gx = np.zeros((3,3))

    # if abs(w) < 1e-5:
    Gx[0, 2] = -v * math.sin(theta) * t
    Gx[1, 2] =  v * math.cos(theta) * t

    # else:
    #     Gx[0, 2] = -(v / w) * math.cos(theta) + (v / w) * math.cos(theta + w * t)
    #     Gx[1, 2] = -(v / w) * math.sin(theta) + (v / w) * math.sin(theta + w * t)

    Gt = np.eye(2*n+3) + Fx.T @ Gx @ Fx 
    print("Gt\n",Gt)

    return Gt


def covariance(Gt, cov, Fx, Rt, n):
    """
    Covariance matrix prediction stage

    Arguments:
    -Gt: Jacobian matrix of motion
    -cov: covariance matrix of the last update
    -Fx : matrix Fx
    -Rt: noise matrix
    -n: total number of landmarks

    Returns:
    -returns the predicted covariance matrix
    """
    # print("Gt:\n" ,Gt)
    # print("matriz 1: ", Gt @ cov @ Gt.T)
    # print("matriz 2: ", Rt*Fx.T @ Fx)
    print("Cov\n",cov)
    p_cov = Gt @ cov @ Gt.T + Rt*Fx.T @ Fx
    print("P_cov\n",p_cov)
    return p_cov