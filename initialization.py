import math
import numpy as np
#Number of landmarks
from main import num_landmarks

#Vetor de estado initializada a 0 
state_vetor = np.zeros((2*num_landmarks + 3, 1))
print(state_vetor)

#Matriz da convariancia
conv_matrix = np.zeros((2*num_landmarks+3,2*num_landmarks+3))
for i in range(3,2*num_landmarks + 3):
    conv_matrix[i,i] = np.inf

print(conv_matrix)