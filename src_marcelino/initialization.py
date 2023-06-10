import numpy as np


def initialize_state_covariance(num_landmarks):
    """
    Inicializa o vetor de estado e a matriz de covariância do EKF SLAM.

    Argumentos:
    - num_landmarks: número de landmarks no ambiente

    Retorna:
    - state_vector: vetor de estado inicializado com zeros
    - covariance_matrix: matriz de covariância inicializada com zeros e diagonal infinita para as landmarks
    """
    state_vector = np.zeros((2 * num_landmarks + 3, 1))

    covariance_matrix = np.zeros((2 * num_landmarks + 3, 2 * num_landmarks + 3))
    for i in range(3, 2 * num_landmarks + 3):
        covariance_matrix[i, i] = 1

    return state_vector, covariance_matrix
