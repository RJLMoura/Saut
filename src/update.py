import math
import numpy as np

def initialize_landmark(i,n,cov,state,obs_z):

    """"
    Etapa da correção

    Argumentos:
    -i: número da landmark a ser atualizada
    -n: número total de landmarks
    -cov: matriz da covariância da última atualização
    -state: vetor de estados
    -obs_z: vetor da distância do robô com a landmark a ser atualizada (r,angulo)

    Retorna:
    -retorna o vetor de estados e a matriz da covariância ajustada
    """
                
    delta = np.zeros((2,1))
    delta[0,0] = state[2*i+3,0] - state[0,0]
    delta[1,0] = state[2*i+4,0] - state[1,0]

    q = np.dot(delta.T, delta)
    pr_z = np.zeros((2,1))
    pr_z[0,0] = math.sqrt(q)
    pr_z[1,0] = math.atan2(delta[1,0],delta[0,0]) - state[2,0]

    Fx = np.zeros((5,3 + (2*i - 2) + 2 +(2*n- 2*i)))
    for i in range(3):
        Fx[i,i] = 1
    Fx[2+(2*i-2)+1,3] = 1
    Fx[2+(2*i-2)+2,4] = 1

    Ht = np.array([(-math.sqrt(q)*delta[0,0], -math.sqrt(q)*delta[1,0], 0, math.sqrt(q)*delta[0,0], math.sqrt(q)*delta[1,0]),
                  (delta[1,0], -delta[0,0], -q, -delta[1,0], delta[0,0])]) / q @ Fx

    Qt = np.eye(2)  # Covariance matrix of observation noise
    K = cov @ Ht.T @ np.linalg.inv(Ht @ cov @ Ht.T + Qt)
    state = state + K @ (obs_z - pr_z)
    cov = (np.eye(2*n+3) - K @ Ht) @ cov

    return state, cov