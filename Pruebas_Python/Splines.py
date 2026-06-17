import numpy as np

def obtener_splines(t, q):
    t = np.asarray(t)
    q = np.asarray(q)
    n = len(t)
    
    h = np.diff(t)
    
    diag_sup_inf = h[1:n-2]
    diag_princ = 2 * (h[:-1] + h[1:])
    
    M = np.diag(diag_princ) + np.diag(diag_sup_inf, 1) + np.diag(diag_sup_inf, -1)
    
    f = np.diff(q) / h
    b_vec = 6 * np.diff(f)
    
    g_mid = np.linalg.solve(M, b_vec)
    
    g = np.concatenate(([0], g_mid, [0]))
    
    a = np.diff(g) / (6 * h)
    b = g[:-1] / 2
    c = f - h * (2 * g[:-1] + g[1:]) / 6
    d = q[:-1]
    
    return np.column_stack((a, b, c, d))

def evaluar_splines_c(S, t, time):
    n = len(t)

    if time >= t[-1]:
        k = n - 2  
    else:
        indices = np.where((t[:-1] <= time) & (time <= t[1:]))[0]
        k = indices[0] if indices.size > 0 else 0

    dt = time - t[k]
    
    coeffs = S[k, :]

    qk = coeffs[0]*dt**3 + coeffs[1]*dt**2 + coeffs[2]*dt + coeffs[3]
    
    qpk = 3*coeffs[0]*dt**2 + 2*coeffs[1]*dt + coeffs[2]

    return qk, qpk

def normlzr_dist(puntos_arr, I):
    diffs = np.diff(puntos_arr, axis=1)
    dist_segmentos = np.sqrt(np.sum(diffs**2, axis=0))
    dist_acumulada = np.concatenate(([0], np.cumsum(dist_segmentos)))
    
    dist_equidistante = np.linspace(0, dist_acumulada[-1], I)
    
    x_eq = np.interp(dist_equidistante, dist_acumulada, puntos_arr[0, :])
    y_eq = np.interp(dist_equidistante, dist_acumulada, puntos_arr[1, :])
    
    return x_eq, y_eq