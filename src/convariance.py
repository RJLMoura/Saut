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
    """
    Jacobiana do movimento obtida a partir da derivada da posição mais a matriz da translação

    Argumentos:
    -v: velocidade atual do robô
    -w: velocidade angular atual do robô
    -t: intervalo de tempo desde da última atualização
    theta: orientação atual do robô   
    Fx: matrix Fx
    n: número total de landmarks

    Retorna:
    -retorna a matriz jacobiana do movimento Gt
    """

    Gx = np.zeros((3,3))
    Gx[0,2] = -(v/w) * math.cos(theta) + (v/w) * math.cos(theta + w*t)
    Gx[1,2] = -(v/w) * math.sin(theta) + (v/w) * math.sin(theta + w*t)

    Gt = np.identity(2*n+3)
    Gt = Gt + list(zip(*Fx)) @ Gx @ Fx 

    return Gt

def covariance(Gt,cov,Fx,Rt,n):

    """Etapa da previsão da matriz da covariância
    
    Argumentos:
    -Gt: matriz jacobiana do movimento
    -cov: matriz da covariância da última atualização
    -Fx : matriz Fx
    -Rt: matriz do ruído
    -n: número total de landmarks

    Retorna:
    -retorna a matriz da covariância prevista
    """


    p_cov = np.zeros((2*n+3,2*n+3))
    p_cov = Gt @ cov @ list(zip(*Gt)) + list(zip(*Fx)) @ Rt @ Fx
    return p_cov
