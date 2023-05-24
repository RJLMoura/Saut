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


    "Se a landmark nunca foi observada"
    if np.isinf(cov[i+3,i+3]):
        state[i+3,0] = state[0,0] + obs_z[0,0] * math.cos(obs_z[1,0] + state[2,0])
        state[i+4,0] = state[1,0] + obs_z[0,0] * math.cos(obs_z[1,0] + state[2,0])
                
    delta = np.zeros(2,1)
    delta[0,0] = state[i+3,0] - state[0,0]
    delta[1,0] = state[i+4,0] - state[1,0]

    q = list(zip(*delta)) @ delta
    pr_z = n.zeros(2,1)
    pr_z[0,0] = math.sqrt(q)
    pr_z[1,0] = math.atan2(delta[0,0],delta[1,0]) - state[2,0]

    Fx = np.zeros((5,3 + (2*i - 2) + + 2 +(2*n- 2*i)))
    for i in range(3):
        Fx[i,i] = 1
    Fx[2+(2*i-2)+1,3] = 1
    Fx[2+(2*i-2)+2,4] = 1

    Ht = (np.array[(-math.sqrt(q)*delta[0,0], -math.sqrt(q)*delta[1,0], 0, math.sqrt(q)*delta[0,0], math.sqrt(q)*delta[1,0]),
                  (delta[1,0], -delta[0,0], -q, -delta[1,0], delta[0,0])] / q) @ Fx
    
    K = cov @ list(zip(*Ht)) @ np.linalg((Ht @ cov @ list(zip(*Ht)) + Qt))
    state = state + K @ (obs_z - pr_z)
    cov = (np.identity(2*n+3) - K @ Ht) @ cov

    return state, cov