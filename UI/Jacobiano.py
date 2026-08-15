import numpy as np
import math as F

def Jacobiano(q):
    a1 = 20
    a2 = 20
    
    q1 = q[0]
    q2 = q[1]
    
    A1 = np.array([
    [F.cos(q1),  -F.sin(q1),     0,     a1*F.cos(q1)],
    [F.sin(q1),  F.cos(q1),      0,     a1*F.sin(q1)],
    [   0,           0,          1,          0],
    [   0,           0,          0,          1]
    ])
    
    A2 = np.array([
    [F.cos(q2),  -F.sin(q2),     0,     a2*F.cos(q2)],
    [F.sin(q2),  F.cos(q2),      0,     a2*F.sin(q2)],
    [   0,           0,          1,          0],
    [   0,           0,          0,          1]
    ])
    
    # Tomar vector de traslación
    A12 =  A1 @ A2
    # Definir el origen Oi-1
    O2 = A12[0:3,3]
    O1 = A1[0:3,3]
    # Tomar columnas Zi-1
    Z1 = A12[0:3,2]
    Z0 = A1[0:3,2]
    
    J1 = np.reshape(np.cross(Z0,O2), (3,1))
    J2 = np.reshape(np.cross(Z1,(O2-O1)), (3,1))
    
    Jacob = np.block([
        [J1,  J2],
        [np.reshape(Z0, (3,1)),  np.reshape(Z1, (3,1))],
    ])
    return O1[:2], O2[:2], Jacob