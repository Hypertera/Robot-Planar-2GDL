import numpy as np
from tkinter import messagebox

def Jacobiano(q):
    a1 = 20 #cm
    a2 = 20 #cm
    
    q1 = q[0]
    q2 = q[1]
    
    A1 = np.array([
    [np.cos(q1),  -np.sin(q1),     0,     a1*np.cos(q1)],
    [np.sin(q1),  np.cos(q1),      0,     a1*np.sin(q1)],
    [   0,           0,          1,          0],
    [   0,           0,          0,          1]
    ])
    
    A2 = np.array([
    [np.cos(q2),  -np.sin(q2),     0,     a2*np.cos(q2)],
    [np.sin(q2),  np.cos(q2),      0,     a2*np.sin(q2)],
    [   0,           0,          1,          0],
    [   0,           0,          0,          1]
    ])
    
    A12 =  A1 @ A2
    
    O2 = A12[0:3,3]
    O1 = A1[0:3,3]
    
    Z1 = A12[0:3,2]
    Z0 = A1[0:3,2]
    
    J1 = np.reshape(np.cross(Z0,O2), (3,1))
    J2 = np.reshape(np.cross(Z1,(O2-O1)), (3,1))
    
    Jacob = np.block([
        [J1,  J2],
        [np.reshape(Z0, (3,1)),  np.reshape(Z1, (3,1))],
    ])
    return O1[:2], O2[:2], Jacob

def CinematicaInversa(x, y, arriba=False):
    
    L1 = 20 #cm
    L2 = 20 #cm
    r2 = x**2 + y**2

    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(c2) > 1:
        messagebox.showwarning("Aviso","Punto fuera del espacio de trabajo")
        q1 = None
        q2 = None
        return q1, q2

    s2 = np.sqrt(1 - c2**2)

    if arriba:
        s2 = -s2

    q2 = np.arctan2(s2, c2)

    q1 = np.arctan2(y, x) - np.arctan2(
        L2 * s2,
        L1 + L2 * c2
    )

    return q1, q2