import numpy as np
import matplotlib.pyplot as plt
from Robot import CinematicaInversa, Jacobiano
from Splines import (obtener_splines, 
                     evaluar_splines_c, 
                     normlzr_dist)

# =============================================================================
# Parámetros
# =============================================================================

S = 8 # Segundos

kx, ky= 5, 5 # Ganancias

xy = np.load('.npy') # Cargar trayectoria

# Elegir pose inicial

# Punto inicial de la trayectoria
q = CinematicaInversa(xy[0][0], xy[1][0]) 

# Punto arbitrario
# q = CinematicaInversa(10, 10)


# %%
# =============================================================================
# Simulación
# =============================================================================

if q[0] is None:
    q = CinematicaInversa(xy[0][0], xy[1][0])

q = np.array(q)
i = xy.shape[1]

qx, qy = normlzr_dist(xy, i)

t = np.linspace(0, S, i)

Spx = obtener_splines(t, qx)
Spy = obtener_splines(t, qy) 

dt = 0.05
c = 0
N = int(round((S+dt)/dt))

color = [1.0, 0.5, 0.0]

Ks = np.array([kx, ky])
D = np.diag(Ks)

q_plot = np.zeros((2, N))
trayctr_plot = np.zeros((2, N))
qp_plot = np.zeros((2, N))
e_plot = np.zeros((2, N))
t_plot = np.zeros((1, N))
robot_plot = np.zeros((2,3))

# Bucle de simulación
for tk in np.arange(0, S+dt, dt):
    
    print(tk)
    
    T1, T2, J = Jacobiano(q)
    
    qkx, qpkx = evaluar_splines_c(Spx, t, tk)
    qky, qpky = evaluar_splines_c(Spy, t, tk)
    
    xyd = np.asarray((qkx,qky))
    xypd = np.asarray((qpkx,qpky))

    # Control
    Ji = np.linalg.pinv(J[0:2,:])
    
    e = xyd - T2
    qp = Ji @ (xypd + D @ (e))

    q = q + qp * dt

    # Guardar valores
    t_plot[:,c] = tk
    q_plot[:, c] = T2
    trayctr_plot[:, c] = xyd
    e_plot[:,c] = e
    qp_plot[:,c] = qp
    robot_plot[:,1] = np.asarray((T1[0], T1[1]))
    robot_plot[:,2] = np.asarray((T2[0], T2[1]))
    
    c = c+1
    
    # Animación
    plt.cla()
    plt.plot(q_plot[0,:c], q_plot[1,:c], linewidth=2.5, color=color, linestyle='--')
    plt.plot(trayctr_plot[0,:c], trayctr_plot[1,:c], 'b', linewidth=1)
    plt.scatter(robot_plot[0,:2], robot_plot[1,:2], color = 'black', s=150)
    plt.plot(robot_plot[0,:], robot_plot[1,:], color = 'black')

    # plt.grid()
    plt.axis('equal')
    plt.pause(0.01)

# %%
# =============================================================================
# Gráficas
# =============================================================================

#%% Trayectoria
plt.figure(num=1)
plt.cla()

plt.title("Trayectoria del robot")
plt.xlabel("x (cm)")
plt.ylabel("y (cm)")

plt.plot(q_plot[0,:c], q_plot[1,:c], linewidth=2.5, color=color, linestyle='--', label="Robot")
plt.plot(trayctr_plot[0,:c], trayctr_plot[1,:c], 'b', linewidth=1, label="Trayectoria")
plt.scatter(robot_plot[0,:2], robot_plot[1,:2], color = 'black', s=150)
plt.plot(robot_plot[0,:], robot_plot[1,:], color = 'black')

plt.grid()
plt.legend()
plt.axis('equal')
plt.show()

# %% Velocidades articulares
fig, axs = plt.subplots(2, 1, sharex=True)

axs[0].plot(t_plot[0,:c], qp_plot[0,:c], 'r-', linewidth=1.5, label="M1")
axs[0].set_ylabel("rad/s")
axs[0].set_title("Art Vel")
axs[0].grid()
axs[0].legend()

axs[1].plot(t_plot[0,:c], qp_plot[1,:c], 'r-', linewidth=1.5, label="M2")
axs[1].set_ylabel("rad/s")
axs[1].set_xlabel("tiempo [s]")
axs[1].grid()
axs[1].legend()

plt.tight_layout()
plt.show()

#%% Error
plt.figure(num=3)

plt.title("Error")
plt.xlabel("tiempo")
plt.ylabel("cm")

plt.plot(t_plot[0,:c], e_plot[0,0:c],'r-', linewidth=1.5, label="ex")
plt.plot(t_plot[0,:c], e_plot[1,:c],'b-', linewidth=1.5, label="ey")

plt.grid()
plt.legend()
# plt.axis('equal')
plt.show()