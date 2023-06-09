import math
import numpy as np

def initialize_landmark(i,n,cov,state, position, obs_z):

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

    print("\nPosition:\n", position)
                
    delta = np.zeros((2,1))
    delta[0,0] = state[2*i+3,0] - position[0,0]
    delta[1,0] = state[2*i+4,0] - position[1,0]

    # print("delta:", delta)

    q_matrix = delta.T @ delta

    q=q_matrix[0, 0]
    # print("q_matrix: ", q_matrix)
    pr_z = np.zeros((2,1))
    pr_z[0,0] = math.sqrt(q)
    pr_z[1,0] = math.atan2(delta[1,0],delta[0,0]) - position[2,0]
    
    # print("q: ", q)

    Fx = np.zeros((5,3 + (2*i - 2) + 2 +(2*n- 2*i)))
    for i in range(3):
        Fx[i,i] = 1
    Fx[3,2+(2*i-2)+1] = 1
    Fx[4,2+(2*i-2)+2] = 1

    Ht = 1/q*np.array([(-math.sqrt(q)*delta[0,0], -math.sqrt(q)*delta[1,0], 0, math.sqrt(q)*delta[0,0], math.sqrt(q)*delta[1,0]),
                  (delta[1,0], -delta[0,0], -q, -delta[1,0], delta[0,0])]) @ Fx

    print("\nHt\n",Ht)
    Qt = [(0.05,0),(0,0.05)]  # Covariance matrix of observation noise

    matrix1=cov@Ht.T
    # print("Ht:\n",Ht.shape)
    # print("Cov:\n",cov.shape)
    # print("Ht.T:\n",Ht.T.shape)
    matrix3 = Ht @ cov @ Ht.T
    # print("Matriz3:\n",matrix3.shape)
    matrix4 = matrix3 + Qt
    # print("Ht\n",Ht)
    matrix2=(Ht @ cov @ Ht.T + Qt)
    # print("Matriz2: \n",matrix2)
    determinant = np.linalg.det(matrix2)

    if abs(determinant) < 1e-15:
        print("\nMatrix is singular or close to singular. Cannot compute inverse.\n")
        matrix2 += 1e-9 * np.eye(matrix2.shape[0])  # Add a small constant to the diagonal
    else:
        print("\n\nit is all good bro :)\n\n")

    K = cov @ Ht.T @ np.linalg.pinv(matrix2)

    # print("\nfirst state\n: ", state)
    # print("\nObs_z\n",obs_z)
    # print("\nPr_z\n",pr_z)
    print("\nK\n",K)
    # print("\nK @ (obs_z - pr_z)\n", K @ (obs_z - pr_z))
    state = state + K @ (obs_z - pr_z)
    # print("\nsecond state:\n ", state)
    cov = (np.eye(2*n+3) - K @ Ht) @ cov

    position = np.array([state[0, 0], state[1, 0], state[2, 0]])
    return cov, state, position