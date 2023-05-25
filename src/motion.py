import math
from main import num_landmarks
import numpy as np


def update_pose(state,Fx, dt, v,n, omega):
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
    new_state = np.zeros(2*n+3,1)
    array = [(-(v/omega)*math.sin(state[2,0]) + (v/omega)*math.sin(state[2,0] + omega*dt)),
             ((v/omega)*math.cos(state[2,0]) - (v/omega)*math.cos(state[2,0] + omega*dt)),
             (omega*dt)]
    new_state = state + list(zip(*Fx)) @ array
    
    return new_state