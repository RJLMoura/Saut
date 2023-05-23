import math
from main import num_landmarks
import numpy as np

Fx = np.zeros((2*num_landmarks + 3, 3))
for i in range(3):
    Fx[i,i] = 1

def update_pose(x, y, theta, dt, v, omega):
    """
    Atualiza a pose (posição e orientação) do robô com base nos comandos de controle.
    
    Argumentos:
    - x: posição x atual do robô
    - y: posição y atual do robô
    - theta: orientação atual do robô (em radianos)
    - dt: intervalo de tempo desde a última atualização
    - v: velocidade linear do robô
    - omega: velocidade angular do robô
    
    Retorna:
    - Novas coordenadas x, y e orientação theta atualizadas
    """
    x_new = x + dt * v * math.cos(theta)
    y_new = y + dt * v * math.sin(theta)
    theta_new = theta + dt * omega
    
    return x_new, y_new, theta_new

    