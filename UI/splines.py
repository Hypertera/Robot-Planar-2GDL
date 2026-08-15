import numpy as np

def obtener_splines(t, q):
    t = np.asarray(t)
    q = np.asarray(q)
    n = len(t)
    
    h = np.diff(t)
    
    # Construcción de la matriz tridiagonal M
    diag_sup_inf = h[1:n-2]
    diag_princ = 2 * (h[:-1] + h[1:])
    
    M = np.diag(diag_princ) + np.diag(diag_sup_inf, 1) + np.diag(diag_sup_inf, -1)
    
    # Vector de términos independientes
    f = np.diff(q) / h
    b_vec = 6 * np.diff(f)
    
    # Resolver para g (segundas derivadas / 2)
    g_mid = np.linalg.solve(M, b_vec)
    
    # Aplicar condiciones de frontera natural (0 en los extremos)
    g = np.concatenate(([0], g_mid, [0]))
    
    # Calcular coeficientes de cada tramo
    a = np.diff(g) / (6 * h)
    b = g[:-1] / 2
    c = f - h * (2 * g[:-1] + g[1:]) / 6
    d = q[:-1]
    
    # Retorna una matriz donde cada fila son los coeficientes [a, b, c, d] del tramo
    return np.column_stack((a, b, c, d))

def evaluar_splines_c(S, t, time):
    n = len(t)

    # Identificar el índice del tramo
    if time >= t[-1]:
        k = n - 2  # Python usa índices desde 0, el último tramo es n-2
    else:
        # Busca el primer índice donde el tiempo encaja en el intervalo [t_k, t_{k+1}]
        # t[1:] es t_{k+1} y t[:-1] es t_k
        indices = np.where((t[:-1] <= time) & (time <= t[1:]))[0]
        k = indices[0] if indices.size > 0 else 0

    # Diferencia de tiempo respecto al inicio del tramo
    dt = time - t[k]
    
    # Coeficientes del tramo k: [a, b, c, d]
    coeffs = S[k, :]

    # Evaluar posición (qk)
    # S[k,0]*dt^3 + S[k,1]*dt^2 + S[k,2]*dt + S[k,3]
    qk = coeffs[0]*dt**3 + coeffs[1]*dt**2 + coeffs[2]*dt + coeffs[3]
    
    # Evaluar velocidad (qpk) - Derivada
    # 3*S[k,0]*dt^2 + 2*S[k,1]*dt + S[k,2]
    qpk = 3*coeffs[0]*dt**2 + 2*coeffs[1]*dt + coeffs[2]

    return qk, qpk

# def evaluar_splines_c(S, t, time):
#     n = len(t)

#     # 1. Manejo de límites: si el tiempo es igual o mayor al final, usar el último tramo
#     if time >= t[-1]:
#         k = n - 2
#         dt = t[-1] - t[k] # Evaluar exactamente en el final del tramo
#     elif time <= t[0]:
#         k = 0
#         dt = 0
#     else:
#         # 2. Buscar el tramo correspondiente
#         k = np.searchsorted(t, time) - 1
#         # Asegurar que k no se salga de los límites por precisión
#         k = max(0, min(k, n - 2))
#         dt = time - t[k]
    
#     coeffs = S[k, :]

#     # Evaluar posición
#     qk = coeffs[0]*dt**3 + coeffs[1]*dt**2 + coeffs[2]*dt + coeffs[3]
    
#     # Evaluar velocidad
#     qpk = 3*coeffs[0]*dt**2 + 2*coeffs[1]*dt + coeffs[2]

#     return qk, qpk


def cinematicaInversa(x, y, elbow_up=False):
    
    # x = x
    # y = y

    L1 = 20
    L2 = 20
    r2 = x**2 + y**2

    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(c2) > 1:
        # messagebox.showwarning("Aviso","Punto fuera del espacio de trabajo")
        q1 = None
        q2 = None
        return q1, q2

    s2 = np.sqrt(1 - c2**2)

    if elbow_up:
        s2 = -s2

    q2 = np.arctan2(s2, c2)

    q1 = np.arctan2(y, x) - np.arctan2(
        L2 * s2,
        L1 + L2 * c2
    )

    return q1, q2

# def calcular_Trayectoria(P0, P1, P2, P3, bdt, S, i, xyhf):
#     # Definición de la matriz de Bézier cúbica
#     M = np.array([
#         [-1,  3, -3,  1],
#         [ 3, -6,  3,  0],
#         [-3,  3,  0,  0],
#         [ 1,  0,  0,  0]
#     ])
    

#     P = np.column_stack((P0, P1, P2, P3))
    

#     tiempos = np.arange(0, S + (bdt / 10), bdt)
    
#     for t in tiempos:
#         tn = t / S
        
#         # Vector de potencias [tn^3, tn^2, tn, 1]
#         T = np.array([tn**3, tn**2, tn, 1])
        

#         xy = P @ M @ T

#         xyhf[:, i] = xy
        
#         i += 1
        
#     return i, xyhf

def normlzr_dist(puntos_arr, I):
    diffs = np.diff(puntos_arr, axis=1)
    dist_segmentos = np.sqrt(np.sum(diffs**2, axis=0))
    dist_acumulada = np.concatenate(([0], np.cumsum(dist_segmentos)))
    
    dist_equidistante = np.linspace(0, dist_acumulada[-1], I)
    
    x_eq = np.interp(dist_equidistante, dist_acumulada, puntos_arr[0, :])
    y_eq = np.interp(dist_equidistante, dist_acumulada, puntos_arr[1, :])
    
    return x_eq, y_eq

import matplotlib.pyplot as plt

def Dibujar_Movil_4N(p, Color, Marc):
    x, y, theta = p

    Lr = 0.2
    l = Lr * 0.1
    L = Lr * 0.3

    phi = np.linspace(0, 2*np.pi, 50)

    # Base
    cx = x + (Lr - l) * np.cos(phi)
    cy = y + (Lr - l) * np.sin(phi)
    plt.plot(cx, cy, linewidth=2, color=Color)

    # Transformación base
    Tob = np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta),  np.cos(theta), y],
        [0, 0, 1]
    ])

    # Marcador delantero
    Tbf = np.array([
        [1, 0, (Lr * 0.5)],
        [0, 1, 0],
        [0, 0, 1]
    ])

    Tor = Tob @ Tbf

    cx = Tor[0, 2] + (Lr * 0.2) * np.cos(phi)
    cy = Tor[1, 2] + (Lr * 0.2) * np.sin(phi)
    plt.plot(cx, cy, linewidth=2, color=Marc)

    # Ruedas
    Tbl = np.array([[1,0,0],[0,1,Lr],[0,0,1]])
    Tbr = np.array([[1,0,0],[0,1,-Lr],[0,0,1]])

    Tol = Tob @ Tbl
    Tor = Tob @ Tbr

    def rect(T):
        p1 = T @ np.array([+L, -l, 1])
        p2 = T @ np.array([-L, -l, 1])
        p3 = T @ np.array([+L, +l, 1])
        p4 = T @ np.array([-L, +l, 1])

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], linewidth=2, color=Color)
        plt.plot([p1[0], p3[0]], [p1[1], p3[1]], linewidth=2, color=Color)
        plt.plot([p2[0], p4[0]], [p2[1], p4[1]], linewidth=2, color=Color)
        plt.plot([p3[0], p4[0]], [p3[1], p4[1]], linewidth=2, color=Color)

    rect(Tol)
    rect(Tor)
