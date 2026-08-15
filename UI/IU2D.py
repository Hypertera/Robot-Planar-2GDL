import customtkinter as ctk
from tkinter import messagebox, filedialog
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import MultipleLocator
import numpy as np
import time
from Comandos import NANO, UNO
import json
import matplotlib.pyplot as plt
from CalcSimul import (calcular_simul, 
                      cinematicaInversa, 
                      Jacobiano,
                      obtener_splines,
                      evaluar_splines_c, 
                      normlzr_dist)

# =============================================================================
# Variables Globales
# =============================================================================

class Variables:

    limx = [-10,45]
    limy = [-45,45]
    punto_espcd = [10, 5]
    puntos_x = []
    puntos_y = []
    
    Robot = 40
    base = 13
    t = np.linspace(0, np.deg2rad(100), 250)
    rx = Robot*np.cos(t)
    ry = Robot*np.sin(t)
    rbx = base*np.cos(t)
    rby = base*np.sin(t)
    
    base_plot = None
    T1 = None
    T2 = None
    
    bezier = None
    xy = None
    t_plot = None 
    q0_plot = None
    q1_plot = None
    trayctr_plot = None
    e_plot = None
    qp_plot = None
    c = 0
    
    esta_conectado = False
    bandera1 = False
    bandera2 = False
    
    sen = NANO()
    mot = UNO()

varbls = Variables()
        
# =============================================================================
# Creación de la ventana
# =============================================================================

def Interfaz():
    
    ctk.set_appearance_mode("Dark")
    ctk.set_default_color_theme("dark-blue")
    
    raiz = ctk.CTk()
    
    raiz.title('Interfaz')
    raiz.geometry('850x650')
    
    color_grid = "#c0c0c0"
    color_txt = "#c0c0c0"
    
    raiz.grid_columnconfigure(0, weight=1)
    raiz.grid_rowconfigure(0, weight=1)
    
    tabs = ctk.CTkTabview(raiz,
                        width=800, 
                        height=600, 
                        corner_radius=20, 
                        fg_color='#1E1E1E', 
                        segmented_button_fg_color='#1E1E1E')
    
    tabs.grid(row=0, column=0, padx=10, pady=10, sticky='nsew')
    
    tabs.add("Gráfica")
    tabs.add("Simulación")
    tabs.add("Controles")
    tabs.add("Opciones")
    
    activo = ctk.IntVar(value=True) # Variable para switch en tab_opciones
    codo = ctk.IntVar(value=False) # Variable para switch en tab_controls
    
    # =============================================================================
    # PESTAÑA: Gráfica
    # =============================================================================

    def crear_tab_graf():
        
        tab_grafica = tabs.tab("Gráfica")
        tab_grafica.grid_columnconfigure(1, weight=1)
        tab_grafica.grid_rowconfigure(0, weight=1)
        
        # =============================================================================
        #       Barra Lateral (Controles) 
        # =============================================================================
        
        sidebar = ctk.CTkFrame(tab_grafica, 
                               width=250, 
                               corner_radius=20, 
                               fg_color='#252526')
        
        sidebar.grid(row=0, column=0, padx=2, pady=2, sticky='nsew')
        
        ctk.CTkLabel(sidebar, 
                     text="Veloz", 
                     font=("Arial Rounded MT Bold", 20)).pack(pady=20)
        
        entries = {}
        row_frame = ctk.CTkFrame(sidebar, fg_color="transparent")
        row_frame.pack(fill="x", padx=0, pady=5)
        
        for axis in ['X', 'Y']:
            
            ctk.CTkLabel(row_frame, 
                         text=f"{axis}:", 
                         width=25, 
                         font=('Arial Rounded MT Bold', 15)).pack(side="left", 
                                                                  padx=(0, 0))
            
            
            entry = ctk.CTkEntry(row_frame, 
                                  width=60, 
                                  border_color='gray',
                                  placeholder_text="0.0")
            entry.pack(side="left", padx=(0, 0))
            
            entries[axis.lower()] = entry
            
        # =============================================================================
        #       Panel de la Gráfica
        # =============================================================================
        
        frame_plot = ctk.CTkFrame(tab_grafica, corner_radius=20, 
                                  fg_color='#252526')
        
        frame_plot.grid(row=0, column=1, padx=10, sticky='nsew')
        
        fig = Figure(figsize=(5, 5), dpi=150, facecolor='#252526')
        
        ax = fig.add_subplot(111, facecolor='#252526')

        canvas_matplotlib = FigureCanvasTkAgg(fig, master=frame_plot)
        canvas_widget = canvas_matplotlib.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=20, pady=1)
        
        # =============================================================================
        # Funciones
        # =============================================================================
        
        def configurar_ejes():
            
            ax.clear()
            color_grid = "#c0c0c0"
            color_txt = "#c0c0c0"
            
            ax.axhline(0, color='#c0c0c0', linewidth=2, alpha=0.5)
            ax.axvline(0, color='#c0c0c0', linewidth=2, alpha=0.5)
            
            ax.set_xlabel("X (cm)", color=color_txt)
            ax.set_ylabel("Y (cm)", color=color_txt)
            ax.tick_params(colors=color_txt)
            
            ax.set_xlim(varbls.limx[0], varbls.limx[1])
            ax.set_ylim(varbls.limy[0], varbls.limy[1])
            ax.xaxis.set_major_locator(MultipleLocator(varbls.punto_espcd[1]))
            ax.yaxis.set_major_locator(MultipleLocator(varbls.punto_espcd[1]))
            
            ax.grid(True, linestyle='--', color=color_grid, alpha=0.3)
            
            baseRobotx = np.array([varbls.rx[249], varbls.rbx[249]])
            baseRoboty = np.array([varbls.ry[249], varbls.rby[249]])
            
            # Area de trabajo (rx, ry)
            i = 1
            while i != -2:
                
                if i == 0:
                    i = i-1
                    continue
                
                ax.plot(varbls.rx, i*varbls.ry, 'r--', alpha=0.4)
                
                ax.plot(varbls.rbx, i*varbls.rby, 'r--', alpha=0.4)
                
                ax.plot(baseRobotx, i*baseRoboty, 'r--', alpha=0.4, 
                        label='Area de trabajo' if i == 1 else "")
                i = i-1
                
            leg = ax.legend(facecolor='#252526', edgecolor='white')
            for text in leg.get_texts(): text.set_color("white")
            
        def calcular_trayectoria(P0, P1, P2, P3):
            
            bdt = 0.02
            S = 1
            
            M = np.array([[-1,  3, -3, 1], 
                          [ 3, -6,  3, 0], 
                          [-3,  3,  0, 0], 
                          [ 1,  0,  0, 0]])
            
            P = np.column_stack((P0, P1, P2, P3))
            
            tiempos = np.arange(0, S + (bdt / 10), bdt)
            xy = np.zeros((2, len(tiempos)))
            
            i = 0
            for t in tiempos:
                tn = t / S
                T = np.array([tn**3, tn**2, tn, 1])
                pts = P @ M @ T
                xy[:, i] = pts
                i += 1
            return xy
        
        def actualizar_grafica():

            configurar_ejes()
            trayectoria_completa = []
            ax.scatter(varbls.puntos_x, varbls.puntos_y, color='#1ABC9C', 
                       s=varbls.punto_espcd[0], zorder=5)
            
            if len(varbls.puntos_x) > 1 and activo.get():
                ax.plot(varbls.puntos_x, varbls.puntos_y, color="#E67E22", 
                        linestyle='--', linewidth=3, alpha=0.5)
    
            num_puntos = len(varbls.puntos_x)
            if num_puntos >= 4:
                for start_idx in range(0, num_puntos - 3, 3):
                    p_indices = range(start_idx, start_idx + 4)
                    
                    P0 = [varbls.puntos_x[p_indices[0]], 
                          
                          varbls.puntos_y[p_indices[0]]]
                    
                    P1 = [varbls.puntos_x[p_indices[1]], 
                          
                          varbls.puntos_y[p_indices[1]]]
                    
                    P2 = [varbls.puntos_x[p_indices[2]], 
                          
                          varbls.puntos_y[p_indices[2]]]
                    
                    P3 = [varbls.puntos_x[p_indices[3]], 
                          
                          varbls.puntos_y[p_indices[3]]]
    
                    trayectoria = calcular_trayectoria(P0, P1, P2, P3)
                    trayectoria_completa.append(trayectoria)
                    
                    varbls.bezier = np.hstack(trayectoria_completa)
                    ax.plot(trayectoria[0, :], trayectoria[1, :], 
                            color='#3498DB', linewidth=3)
            else:
                varbls.bezier = None
                
            canvas_matplotlib.draw()
    
        def agregar_manual():
            try:
                x = float(entries['x'].get())
                y = float(entries['y'].get())
                
                varbls.puntos_x.append(x)
                varbls.puntos_y.append(y)
                
                actualizar_grafica()
                
                for e in entries.values(): e.delete(0, 'end')
                
            except ValueError:
                messagebox.showerror("Error", "Dato inválido")
                
        def al_hacer_clic(event):
            if event.inaxes:
                varbls.puntos_x.append(round(event.xdata, 2))
                varbls.puntos_y.append(round(event.ydata, 2))
                actualizar_grafica()
                
        def guardar():
            if varbls.bezier is not None:
                
                for i in range(varbls.bezier.shape[1]):
                    
                    w = np.atan2(varbls.bezier[1][i], varbls.bezier[0][i])
                    
                    h = np.sqrt((varbls.bezier[0][i]**2) + (varbls.bezier[1][i]**2))
                    
                    q = cinematicaInversa(varbls.bezier[0][i], varbls.bezier[1][i])
                    
                    if w > np.radians(100) or w < np.radians(-100) or h < 13 or (q is None):
                        messagebox.showwarning("Aviso", 
                                               "Trayectoria Invalida")
                        return
                    
                ruta_archivo = filedialog.asksaveasfilename(
                    defaultextension=".npy",
                    filetypes=[("Archivo NumPy", "*.npy")],
                    title="Guardar trayectoria como..."
                )
                if ruta_archivo:
                    np.save(ruta_archivo, varbls.bezier)
                    messagebox.showinfo("Éxito", f"Trayectoria guardada en:\n{ruta_archivo}")
            else:
                messagebox.showwarning("Aviso", "No hay trayectoria")
                
        def borrar_punto():
            if len(varbls.puntos_x) > 0:
                varbls.puntos_x.pop()
                varbls.puntos_y.pop()
                actualizar_grafica()
                
        def borrar_todo():
            varbls.puntos_x.clear()
            varbls.puntos_y.clear()
            actualizar_grafica()
            
        def cerrar_aplicacion():
            if varbls.esta_conectado:
                varbls.sen.cerrarSerial()
                varbls.mot.cerrarSerial()
            raiz.quit()
            raiz.destroy()
        
        # =============================================================================
        #       Botones
        # =============================================================================
        
        agregar = ctk.CTkButton(sidebar, 
                                text="Añadir Punto", 
                                font=('Segoe UI Semibold', 16), 
                                command=agregar_manual)
    
        agregar.pack(pady=[20,10], padx=20)
    
        fig.canvas.mpl_connect('button_press_event', al_hacer_clic)
    
        guardar = ctk.CTkButton(sidebar, 
                                 text="Guardar", 
                                 font=('Segoe UI Semibold', 16), 
                                 command=guardar)
    
        guardar.pack(pady=10, padx=20)
    
        remover = ctk.CTkButton(sidebar, 
                                text="Remover Punto", 
                                font=('Segoe UI Semibold', 16), 
                                fg_color="red", 
                                hover_color="#E74C3C", 
                                command=borrar_punto)
    
        remover.pack(pady=10, padx=20)
    
        limpiar = ctk.CTkButton(sidebar, 
                                  text="Limpiar Todo", 
                                  font=('Segoe UI Semibold', 16), 
                                  fg_color="red", 
                                  hover_color="#E74C3C", 
                                  command=borrar_todo)
    
        limpiar.pack(pady=10, padx=20)
    
        salir = ctk.CTkButton(sidebar, 
                                 text="Salir", 
                                 font=('Segoe UI Semibold', 16), 
                                 fg_color="transparent", 
                                 border_width=2, border_color='gray', 
                                 command=cerrar_aplicacion)
    
        salir.pack(side="bottom", pady=20, padx=20)
        
        configurar_ejes()
        return actualizar_grafica, cerrar_aplicacion
    
    # =============================================================================
    # PESTAÑA: Simulación
    # =============================================================================
    
    def crear_tab_simul():
        
        tab_simulacion = tabs.tab("Simulación")
        tab_simulacion.grid_columnconfigure(1, weight=1)
        tab_simulacion.grid_rowconfigure(0, weight=1)
                
        # =============================================================================
        #       Panel lateral (controles)
        # =============================================================================
        
        sidebar2 = ctk.CTkFrame(tab_simulacion, 
                               width=250, 
                               corner_radius=20, 
                               fg_color='#252526')
        
        sidebar2.grid(row=0, column=0, padx=2, pady=2, sticky='nsew')
        
        # =============================================================================
        #       Tiempo
        # =============================================================================
        
        ctk.CTkLabel(sidebar2, 
                     text="Tiempo:", 
                     font=("Arial Rounded MT Bold", 15)).pack(pady=2)
        
        entry_t = ctk.CTkEntry(sidebar2, 
                               placeholder_text="Segundos", 
                               border_color='gray', 
                               border_width=2)
        
        frame_plot_simul = ctk.CTkScrollableFrame(tab_simulacion, 
                                                  width=450,
                                                  corner_radius=20, 
                                                  fg_color='#252526')
        
        entry_t.pack(fill="x", padx=(27, 27), pady=(0, 10))
        
        # =============================================================================
        #       Pose del robot
        # =============================================================================
        
        ctk.CTkLabel(sidebar2, 
                     text="Pose del robot:", 
                     font=("Arial Rounded MT Bold", 15)).pack(pady=2)
        
        entries2 = {}
        
        row_frame1 = ctk.CTkFrame(sidebar2, fg_color="transparent")
        row_frame1.pack(fill="x", padx=5, pady=5)
        
        for axis in ['X', 'Y']:

            ctk.CTkLabel(row_frame1, 
                         text=f"{axis}:", 
                         width=25, 
                         font=('Arial Rounded MT Bold', 15)).pack(side="left", 
                                                                  padx=(0, 0))
            
            entry2 = ctk.CTkEntry(row_frame1, 
                                  width=60,
                                  border_color='gray',
                                  placeholder_text="0.0")
            entry2.pack(side="left", padx=(0, 0))
            
            entries2[axis.lower()] = entry2
        
        # =============================================================================
        #       Ganancias
        # =============================================================================
        
        ctk.CTkLabel(sidebar2, 
                     text="Ganancias:", 
                     font=("Arial Rounded MT Bold", 15)).pack(pady=2)
        
        entries3 = {}
        
        row_frame2 = ctk.CTkFrame(sidebar2, fg_color="transparent")
        row_frame2.pack(fill="x", padx=5, pady=5)
        
        for axis2 in ['kx', 'ky']:
            ctk.CTkLabel(row_frame2, 
                         text=f"{axis2}:", 
                         width=25, 
                         font=('Arial Rounded MT Bold', 15)).pack(side="left", 
                                                                  padx=(0, 0))
            
            entry3 = ctk.CTkEntry(row_frame2, 
                                  width=60, 
                                  border_color='gray',
                                  placeholder_text="0.0")
            entry3.pack(side="left", padx=(0, 2))
            
            entries3[axis2.lower()] = entry3
            
        # =============================================================================
        #       Graficas
        # =============================================================================
        
        frame_plot_simul.grid(row=0, column=1, padx=10, sticky='nsew')
        
        fig2 = Figure(figsize=(5, 5), dpi=150, facecolor='#252526')
        Rb = fig2.add_subplot(111, facecolor='#252526') # Robot plot
        
        canvas_Rb = FigureCanvasTkAgg(fig2, master=frame_plot_simul)
        canvas_widget = canvas_Rb.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=20, pady=1)
        
        fig3 = Figure(figsize=(5, 5), dpi=150, facecolor='#252526')
        V1 = fig3.add_subplot(211, facecolor='#252526') # V1 plot
        V2 = fig3.add_subplot(212, facecolor='#252526') # V2 plot
        
        canvas_Vel = FigureCanvasTkAgg(fig3, master=frame_plot_simul)
        canvas_widget = canvas_Vel.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=20, pady=1)
        
        fig4 = Figure(figsize=(5, 5), dpi=150, facecolor='#252526')
        M1 = fig4.add_subplot(211, facecolor='#252526') # M1 plot
        M2 = fig4.add_subplot(212, facecolor='#252526') # M2 plot
        
        canvas_Mot = FigureCanvasTkAgg(fig4, master=frame_plot_simul)
        canvas_widget = canvas_Mot.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=20, pady=1)
        
        fig5 = Figure(figsize=(5, 5), dpi=150, facecolor='#252526')
        Err = fig5.add_subplot(111, facecolor='#252526') # Error plot
        
        canvas_Err = FigureCanvasTkAgg(fig5, master=frame_plot_simul)
        canvas_widget = canvas_Err.get_tk_widget()
        canvas_widget.pack(fill='both', expand=True, padx=20, pady=1)
        
        # =============================================================================
        #       Funciones
        # =============================================================================
        
        def refrescar_simulacion():
            
           Rb.clear()
           # Rb.axis('equal')
    
           Rb.set_title('Trayectoria', color=color_txt, pad=15)
           Rb.set_xlabel("X (cm)", color=color_txt)
           Rb.set_ylabel("Y (cm)", color=color_txt)
           Rb.tick_params(colors=color_txt)
    
           Rb.set_xlim(varbls.limx[0], varbls.limx[1])
           Rb.set_ylim(varbls.limy[0], varbls.limy[1])
           
           Rb.grid(True, linestyle='--', color=color_grid, alpha=0.3)
    
           varbls.base_plot, = Rb.plot([], [], 
                                   marker='o', 
                                   color='#1ABC9C', 
                                   markersize=10) # Base
           
           varbls.T1, = Rb.plot([], [], 
                                marker='o',
                                color='#1ABC9C', 
                                markersize=10) # T1
           
           varbls.T2, = Rb.plot([],[], 
                                 linestyle='-',
                                 color='#1ABC9C',
                                 markersize=10) # T2
           
           # Rb.axis('equal')
           canvas_Rb.draw()
           
           V1.set_title('Velocidades Articulares', color=color_txt)
    
           for ax in (V1, V2, M1, M2):
               
               ax.clear()
    
               ax.set_facecolor('#252526')
    
               ax.tick_params(colors=color_txt)
               
               ax.grid(True,
                       linestyle='--',
                       color=color_grid,
                       alpha=0.3)
    
           V1.set_title('Velocidades Articulares', color=color_txt)
           V1.set_ylabel("rad/s", color=color_txt)
           V2.set_ylabel("rad/s", color=color_txt)
           V2.set_xlabel("Tiempo (s)", color=color_txt)
           
           M1.set_title('Velocidades Articulares', color=color_txt)
           M1.set_title('Velocidades Motores', color=color_txt)
           M1.set_ylabel("rad/s", color=color_txt)
           M2.set_ylabel("rad/s", color=color_txt)
           M2.set_xlabel("Tiempo (s)", color=color_txt)
    
           Err.clear()
           Err.set_title('Error', color=color_txt)
           Err.set_xlabel("tiempo (s)", color=color_txt)
           Err.set_ylabel("cm", color=color_txt)
           Err.tick_params(colors=color_txt)
           Err.grid(True, linestyle='--', color=color_grid, alpha=0.3)
       
        def inicializar_simulacion(i):
            
            varbls.base_plot.set_data([0], [0])
            
            varbls.T1.set_data([0, varbls.q0_plot[0][i]], 
                             [0, varbls.q0_plot[1][i]])
            
            varbls.T2.set_data([varbls.q0_plot[0][i], varbls.q1_plot[0][i]], 
                             [varbls.q0_plot[1][i], varbls.q1_plot[1][i]])
            
            Rb.plot(varbls.trayctr_plot[0][0], varbls.trayctr_plot[1][0], 
                       marker='o', color='#1ABC9C', markersize=3)
            
            Rb.plot(varbls.trayctr_plot[0,:], 
                       varbls.trayctr_plot[1,:], 
                       color="#E67E22", 
                       linestyle='-',
                       alpha=0.5)
            
            canvas_Rb.draw()
            
            V1.plot(varbls.t_plot[0,:], 
                            varbls.qp_plot[0,:], 
                            label='q1')
            
            V2.plot(varbls.t_plot[0,:], 
                            varbls.qp_plot[1,:], 
                            label='q2')
            
            M1.plot(varbls.t_plot[0,:], 
                            varbls.qp_plot[0,:]*4, 
                            label='M1')
            
            M2.plot(varbls.t_plot[0,:], 
                            varbls.qp_plot[1,:]*4, 
                            label='M2')
            
            Err.plot(varbls.t_plot[0,:], 
                       varbls.e_plot[0,:], 
                       label='ex', color='#3498DB')
            
            Err.plot(varbls.t_plot[0,:], 
                       varbls.e_plot[1,:], 
                       label='ey', color='#E67E22')
            
            for ax in [V1, V2, M1, M2, Err]:
                leg = ax.legend(facecolor='#252526', edgecolor='white')
                for text in leg.get_texts():
                    text.set_color("white")
         
            canvas_Vel.draw()
            
            canvas_Mot.draw()
            
            canvas_Err.draw()
       
        def actualizar_simulacion():
            
            if varbls.c >= varbls.trayctr_plot.shape[1]:
                simul.configure(state='enabled')
                result.configure(state='enabled')
                carg.configure(state= 'enabled')
                return
       
            varbls.base_plot.set_data([0], [0])
            
            varbls.T1.set_data([0, varbls.q0_plot[0][varbls.c]],
                               [0, varbls.q0_plot[1][varbls.c]]) 
            
            varbls.T2.set_data([varbls.q0_plot[0][varbls.c],  
                                varbls.q1_plot[0][varbls.c]],
                               
                               [varbls.q0_plot[1][varbls.c], 
                                varbls.q1_plot[1][varbls.c]])
            canvas_Rb.draw()
        
            varbls.c += 1
        
            raiz.after(5, actualizar_simulacion)
            
        def cargar():
            try:
                ruta_archivo = filedialog.askopenfilename(
                    defaultextension=".npy",
                    filetypes=[("Archivo NumPy", "*.npy")],
                    title="cargar trayectoria...")
                varbls.xy = np.load(ruta_archivo)
                
                refrescar_simulacion()
                
                Rb.plot(varbls.xy[0,:], 
                           varbls.xy[1,:], 
                           color="#E67E22", 
                           linestyle='-',
                           alpha=0.5)
                
                Rb.scatter(varbls.xy[0][0], varbls.xy[1][0], 
                           color='#1ABC9C', s=10, 
                           label=f"x:{varbls.xy[0][0]}, y:{varbls.xy[1][0]}")
                
                leg = Rb.legend(facecolor='#252526', edgecolor='white')
                for text in leg.get_texts(): text.set_color("white")
                            
                canvas_Rb.draw()
                
            except:
                messagebox.showwarning("Aviso", 
                                       "No se ha podido cargar el archivo")
        
        def animacion():
            try:
                x = float(entries2['x'].get())
                y = float(entries2['y'].get())
                elbow_up = False
                
                Kx = float(entries3['kx'].get())
                Ky = float(entries3['ky'].get())
                
                s = float(entry_t.get())
                
                if s == 0 or Kx == 0 or Ky == 0:
                    messagebox.showwarning("Aviso", 
                                           "No es posible elegir 0 como segundos o como ganancias, inútil")
                    return
                
                if x < 0 and y < 0:
                    elbow_up = True
                
                q1, q2 = cinematicaInversa(x, y, elbow_up)
                
                if q1 is None:
                    messagebox.showwarning("Aviso","Pose fuera del espacio de trabajo")
                    return
                
                plots = calcular_simul(abs(s), varbls.xy, q1, q2, Kx, Ky)
                
                varbls.t_plot = plots[0]
                varbls.q0_plot = plots[1]
                varbls.q1_plot = plots[2]
                varbls.trayctr_plot = plots[3]
                varbls.e_plot = plots[4] 
                varbls.qp_plot = plots[5]
                
                refrescar_simulacion()
                
                inicializar_simulacion(0)
                
                simul.configure(state='disabled')
                result.configure(state='disabled')
                carg.configure(state= 'disabled')
                
                varbls.c = 0
                raiz.after(1500, actualizar_simulacion)
                
            except ValueError:
                messagebox.showerror("Error", "Dato inválido")
            
        def resultados():
            try:
                x = float(entries2['x'].get())
                y = float(entries2['y'].get())
                elbow_up = False
                
                Kx = float(entries3['kx'].get())
                Ky = float(entries3['ky'].get())
                
                s = float(entry_t.get())
                
                if s == 0:
                    messagebox.showwarning("Aviso", 
                                           "No es posible elegir 0 como segundos o como ganancias, inútil")
                    return
                
                if x < 0 and y < 0:
                    elbow_up = True
                
                q1, q2 = cinematicaInversa(x, y, elbow_up)
                
                if q1 is None:
                    messagebox.showwarning("Aviso","Pose fuera del espacio de trabajo")
                    return
                
                # t_plot, q0_plot, q_plot, trayctr_plot, e_plot, qp_plot
                
                plots = calcular_simul(abs(s), varbls.xy, q1, q2, Kx, Ky)
                
                varbls.t_plot = plots[0]
                varbls.q0_plot = plots[1]
                varbls.q1_plot = plots[2]
                varbls.trayctr_plot = plots[3]
                varbls.e_plot = plots[4] 
                varbls.qp_plot = plots[5]
                
                if type(varbls.t_plot) is int:
                    messagebox.showwarning("Aviso", 
                                           "No se pudo cargar la trayectoria")
                    return
                
                i = varbls.t_plot.shape[1] - 1
                
                refrescar_simulacion()
                
                inicializar_simulacion(i)
                        
                Rb.plot(varbls.q1_plot[0,:], 
                           varbls.q1_plot[1,:], 
                           color="#3498DB", 
                           linestyle='--',
                           linewidth=2.5,
                           alpha=0.5)
                
                canvas_Rb.draw()
                
            except ValueError:
                messagebox.showerror("Error", "Dato inválido")
        
        # =============================================================================
        #       Botones
        # =============================================================================
    
        carg = ctk.CTkButton(sidebar2, 
                                text="Cargar Trayectoria", 
                                font=('Segoe UI Semibold', 14), 
                                command=cargar)
    
        carg.pack(pady=[25,10], padx=20)
    
        simul = ctk.CTkButton(sidebar2, 
                                text="Ver Animación", 
                                font=('Segoe UI Semibold', 16), 
                                command=animacion)
    
        simul.pack(pady=[15,10], padx=20)
    
        result = ctk.CTkButton(sidebar2, 
                                 text="Ver Resultados", 
                                 font=('Segoe UI Semibold', 16), 
                                 command=resultados)
    
        result.pack(pady=10, padx=20)
    
        salir = ctk.CTkButton(sidebar2, 
                                 text="Salir", 
                                 font=('Segoe UI Semibold', 16), 
                                 fg_color="transparent", 
                                 border_width=2, border_color='gray', 
                                 command=cerrar_aplicacion)
    
        salir.pack(side="bottom", pady=20, padx=20)
        
        refrescar_simulacion()

    # =============================================================================
    # PESTAÑA: Controles
    # =============================================================================

    def crear_tab_controls():
        
        tab_controls = tabs.tab("Controles")
        
        for i in range(2):
            tab_controls.grid_columnconfigure(i, weight=1)
        
        tab_controls.grid_rowconfigure(1, weight=1)
        
        frame_superior = ctk.CTkFrame(tab_controls, 
                                      height=60, 
                                      corner_radius=20, 
                                      fg_color='#252526')
        
        frame_superior.grid(row=0, 
                            column=0, 
                            columnspan=2, 
                            sticky="nsew", 
                            padx=10, 
                            pady=10)
        
        frame_superior.grid_propagate(False)
        
        # =============================================================================
        #   Panel Izquierdo
        # =============================================================================

        frame_izquierdo = ctk.CTkFrame(tab_controls, 
                                       corner_radius=20, 
                                       fg_color='#252526')
        
        frame_izquierdo.grid(row=1, 
                             column=0, 
                             sticky="nsew", 
                             padx=(10, 5), 
                             pady=(0, 10))
        
        
        lbl_tit_izq = ctk.CTkLabel(frame_izquierdo, 
                                   text="Trayectoria", 
                                   font=('Segoe UI Semibold', 16))
        
        lbl_tit_izq.pack(pady=10)
            
        # =============================================================================
        #       Tiempo
        # =============================================================================
            
        ctk.CTkLabel(frame_izquierdo, 
                     text="Tiempo:", 
                     font=("Arial Rounded MT Bold", 15)).pack(pady=20)

        
        entry_t = ctk.CTkEntry(frame_izquierdo, 
                               placeholder_text="Segundos", 
                               border_color='gray', 
                               border_width=2)
        
        entry_t.pack(padx=(27, 27), pady=(0, 10))
        
        # =============================================================================
        #       Ganancias
        # =============================================================================
        
        ctk.CTkLabel(frame_izquierdo, 
                     text="Ganancias:", 
                     font=("Arial Rounded MT Bold", 15)).pack(pady=2)
        
        entries3 = {}
        
        row_frame2 = ctk.CTkFrame(frame_izquierdo, fg_color="transparent")
        row_frame2.pack(padx=5, pady=5)
        
        for axis2 in ['kx', 'ky']:
            ctk.CTkLabel(row_frame2, 
                         text=f"{axis2}:", 
                         width=25, 
                         font=('Arial Rounded MT Bold', 15)).pack(side="left", 
                                                                  padx=(0, 0))
            
            entry3 = ctk.CTkEntry(row_frame2, 
                                  width=60, 
                                  border_color='gray',
                                  placeholder_text="0.0")
            entry3.pack(side="left", padx=(0, 2))
            
            entries3[axis2.lower()] = entry3
        
        # =============================================================================
        #   Panel Derecho
        # =============================================================================
        
        frame_derecho = ctk.CTkFrame(tab_controls, 
                                     corner_radius=20, 
                                     fg_color='#252526')
        
        frame_derecho.grid(row=1, 
                           column=1, 
                           sticky="nsew", 
                           padx=(5, 10), 
                           pady=(0, 10))
        
        
        lbl_tit_der = ctk.CTkLabel(frame_derecho, 
                                   text="Controles", 
                                   font=('Segoe UI Semibold', 16))
        
        lbl_tit_der.pack(pady=10)
        
        # =============================================================================
        #  Pose    
        # =============================================================================
        
        ctk.CTkLabel(frame_derecho, 
                     text="Pose del robot:", 
                     font=("Arial Rounded MT Bold", 15)).pack(pady=2)
        
        entries2 = {}
        
        row_frame1 = ctk.CTkFrame(frame_derecho, fg_color="transparent")
        row_frame1.pack(padx=20, pady=0)
        
        switch2 = ctk.CTkSwitch(row_frame1, 
                                width=5,
                                text='Codo Arriba', 
                                font=("Arial Rounded MT Bold", 12),
                                variable=codo, onvalue=1, offvalue=0)
        
        switch2.pack(side='right', padx=5)
        
        for axis in ['X', 'Y']:

            ctk.CTkLabel(row_frame1, 
                         text=f"{axis}:", 
                         width=25, 
                         font=('Arial Rounded MT Bold', 15)).pack(side="left", 
                                                                  padx=(0, 0))
            
            entry2 = ctk.CTkEntry(row_frame1, 
                                  width=60,
                                  border_color='gray',
                                  placeholder_text="0.0")
            
            entry2.pack(side="left", padx=(0, 0))
            
            entries2[axis.lower()] = entry2
            
        
        
        # =============================================================================
        #  Articulación 1    
        # =============================================================================
        
        frame_mot_1 = ctk.CTkFrame(frame_derecho, fg_color='transparent')
        
        frame_mot_1.pack(fill='x', pady=10)
        
        for i in range(3):
            frame_mot_1.grid_columnconfigure(i, weight=1)
        
        for i in range(6):
            frame_mot_1.grid_rowconfigure(i, weight=1)
    
        lbl_mot_1 = ctk.CTkLabel(frame_mot_1, 
                                   text="Articulación 1", 
                                   font=('Segoe UI Semibold', 16))
        
        lbl_mot_1.grid(row=1, column=1)
        
        # =============================================================================
        #  Articulación 2    
        # =============================================================================
        
        frame_mot_2 = ctk.CTkLabel(frame_mot_1, 
                                   text="Articulación 2", 
                                   font=('Segoe UI Semibold', 16))
        
        frame_mot_2.grid(row=2, column=1, pady=20)
        
        # =============================================================================
        #  Calibración    
        # =============================================================================
        
        lbl_tit_der1 = ctk.CTkLabel(frame_mot_1, 
                                   text="Calibración", 
                                   font=('Segoe UI Semibold', 16))
        
        lbl_tit_der1.grid(row=3, column=1)
        
        # =============================================================================
        #   Funciones
        # =============================================================================
        
        def habilitarBtns():
            carg.configure(state= 'enabled')
            ejecutar.configure(state= 'enabled')
            mover.configure(state= 'enabled')
            izq_mot1.configure(state= 'enabled')
            der_mot1.configure(state= 'enabled')
            izq_mot2.configure(state= 'enabled')
            der_mot2.configure(state= 'enabled')
            carg_calib.configure(state= 'enabled')
            calib.configure(state= 'enabled')
            
        def deshabilitarBtns():
            carg.configure(state= 'disabled')
            ejecutar.configure(state= 'disabled')
            mover.configure(state= 'disabled')
            izq_mot1.configure(state= 'disabled')
            der_mot1.configure(state= 'disabled')
            izq_mot2.configure(state= 'disabled')
            der_mot2.configure(state= 'disabled')
            carg_calib.configure(state= 'disabled')
            calib.configure(state= 'disabled')
        
        def esperaConexion():
            if varbls.esta_conectado == False:
                conectar.configure(state= 'disabled')
                conectar.configure(text="Conectando", 
                                   fg_color="#1f538d", 
                                   hover_color="#14375e")
                raiz.after(100, conexion)
                
            else:
                conectar.configure(state= 'disabled')
                conectar.configure(text="Desconectando", 
                                   fg_color="#d32f2f",
                                   hover_color="#E74C3C")
                
                raiz.after(100, conexion)
        
        def conexion():
            
            if varbls.esta_conectado == False:
                
                if varbls.sen.conectarArd() and varbls.mot.conectarArd():
                    
                    conectar.configure(state= 'enabled')
                    
                    conectar.configure(text="Desconectar", 
                                       fg_color="#d32f2f",
                                       hover_color="#E74C3C")
                    
                    estado.configure(text="Robot Conectado", 
                                 font=('Segoe UI Semibold', 16),
                                 text_color="#2e7d32")
                    
                    with open('calibracion.json', 'r', encoding='utf-8') as archivo:
                        x = json.load(archivo)
                        
                    for i in range(2):
                        varbls.sen.enviarCal(x["sensores"][i]["ID"], 
                                             x["sensores"][i]["offset"], 
                                             x["sensores"][i]["invertido"])
                    
                    varbls.esta_conectado = True
                    
                    habilitarBtns()
                    
                else:
                    messagebox.showwarning("Aviso","No se ha encontrado el Arduino")
                    conectar.configure(state= 'enabled')
                    
            else:
                
                varbls.sen.cerrarSerial()
                varbls.mot.cerrarSerial()
                
                deshabilitarBtns()
                
                varbls.esta_conectado = False
                
                conectar.configure(text="Conectar", 
                                   fg_color="#1f538d", 
                                   hover_color="#14375e")
                
                estado.configure(text="Robot Desconectado", 
                                 font=('Segoe UI Semibold', 16),
                                 text_color="#c62828")
                
                conectar.configure(state= 'enabled')
                
        def esperarCargar():
            deshabilitarBtns()
            conectar.configure(state= 'disabled')
            
            raiz.after(100, cargar)
        
        def cargar():
            try:
                ruta_archivo = filedialog.askopenfilename(
                    defaultextension=".npy",
                    filetypes=[("Archivo NumPy", "*.npy")],
                    title="cargar trayectoria...")
                v = np.load(ruta_archivo)
                
                for i in range(v.shape[1]):
                    
                    w = np.atan2(v[1][i], v[0][i])
                    
                    h = np.sqrt((v[0][i]**2) + (v[1][i]**2))
                    
                    q = cinematicaInversa(v[0][i], v[1][i])
                    
                    if w > np.radians(100) or w < np.radians(-100) or h < 13 or (q is None):
                        messagebox.showwarning("Aviso", 
                                               "Trayectoria Invalida")
                        return
                
                varbls.bezier = v
                
                qd = cinematicaInversa(varbls.bezier[0][0], varbls.bezier[1][0])
                
                q = varbls.sen.obtenerPosicion()
                
                qt = calcularAng(q, qd)
                
                varbls.mot.enviarPosArt(qt)
                
                habilitarBtns()
                conectar.configure(state= 'enabled')
                
            except:
                messagebox.showwarning("Aviso", 
                                       "No se ha podido cargar el archivo")
                
                deshabilitarBtns()
                conectar.configure(state= 'disabled')

        def esperarMandarPos():
            deshabilitarBtns()
            conectar.configure(state= 'disabled')
            
            raiz.after(100, mandarPos)
        
        def calcularAng(q, qd):
            qt = [0]*2
            for i in range(len(qt)):
                x = np.cos(np.radians(q[i]))
                y = np.sin(np.radians(q[i]))
                w = np.atan2(y,x)
                qt[i] = w
            
            for i in range(len(qt)):
                qt[i] = qd[i] - qt[i]
                
            return qt
        
        def mandarPos():
            xd = float(entries2['x'].get())
            yd = float(entries2['y'].get())
            
            w = np.atan2(yd,xd)
            
            if np.sqrt(xd**2 + yd**2) < 13 or w > np.radians(100) or w < np.radians(-100):
                messagebox.showwarning("Aviso",
                                       "Punto fuera del espacio de trabajo")
                habilitarBtns()
                conectar.configure(state= 'enabled')
                return
                
            qd = cinematicaInversa(xd, yd, codo.get())
            
            q = varbls.sen.obtenerPosicion()
            
            if qd is None:
                messagebox.showwarning("Aviso",
                                       "Punto fuera del espacio de trabajo")
                habilitarBtns()
            else:
                qt = calcularAng(q, qd)
                    
                varbls.mot.enviarPosArt(qt)
                conectar.configure(state= 'enabled')
                habilitarBtns()
                
        def masM1():
            if varbls.bandera1 is False:
                varbls.mot.enviarVel((1, 0))
                varbls.bandera1 = True
            else:
                varbls.mot.enviarVel((0, 0))
                varbls.bandera1 = False
        
        def menosM1():
            if varbls.bandera1 is False:
                varbls.mot.enviarVel((-1, 0))
                varbls.bandera1 = True
            else:
                varbls.mot.enviarVel((0, 0))
                varbls.bandera1 = False
        
        def masM2():
            if varbls.bandera2 is False:
                varbls.mot.enviarVel((0, 1))
                varbls.bandera2 = True
            else:
                varbls.mot.enviarVel((0, 0))
                varbls.bandera2 = False
        
        def menosM2():
            if varbls.bandera2 is False:
                varbls.mot.enviarVel((0, -1))
                varbls.bandera2 = True
            else:
                varbls.mot.enviarVel((0, 0))
                varbls.bandera2 = False
            
        def cargarCalib():
            try:
                ruta_archivo = filedialog.askopenfilename(
                title="Selecciona un archivo JSON",
                filetypes=[("Archivos JSON", "*.json"), 
                           ("Todos los archivos", "*.*")])
                
                with open(ruta_archivo, 'r', encoding='utf-8') as archivo:
                    x = json.load(archivo)
                    
                for i in range(2):
                    varbls.sen.enviarCal(x["sensores"][i]["ID"], 
                                         x["sensores"][i]["offset"], 
                                         x["sensores"][i]["invertido"])
                    
                messagebox.showinfo("¡Exito!", 
                                       "¡Calibración Cargada!")
                
            except:
                messagebox.showwarning("Aviso", 
                                       "No se ha podido cargar el archivo")
                
        def calibrar():
            try:
                for i in range(2):
                    varbls.sen.enviarCal(i, 0, False)
                
                ang = varbls.sen.obtenerPosicion()
                
                datos = {
                    "sensores": [
                        {"ID": 0, "offset": ang[0], "invertido": False},
                        {"ID": 1, "offset": ang[1], "invertido": True}
                    ]
                }

                               
                ruta_archivo = filedialog.asksaveasfilename(
                    defaultextension=".json",
                    filetypes=[("Archivo JSON", "*.json")],
                    title="Guardar calibración como...")
                
                with open(ruta_archivo, "w") as f:
                    json.dump(datos, f, indent=4)
                    
                messagebox.showinfo("¡Exito!", 
                                       "¡Sensores Calibrados!")
                
            except:
                messagebox.showwarning("Aviso", 
                                       "No se ha podido cargar el archivo")
        
        
        def esperarRobot():
            deshabilitarBtns()
            conectar.configure(state= 'disabled')
            raiz.after(100, correrRobot)
        
        def correrRobot():
            
            k1 = float(entries3['kx'].get())
            k2 = float(entries3['ky'].get())
            
            S = float(entry_t.get())
            dt = 0.05
            
            i = varbls.bezier.shape[1]
            qx,qy = normlzr_dist(varbls.bezier, i)

            t = np.linspace(0, S, i)

            Spx = obtener_splines(t, qx)
            Spy = obtener_splines(t, qy) 

            c = 0
            N = int(round((S+dt)/dt))
            
            Ks = np.array([k1,k2])
            D = np.diag(Ks)
            
            qd = cinematicaInversa(varbls.bezier[0][0], varbls.bezier[1][0])
            
            q = varbls.sen.obtenerPosicion()
            
            qt = calcularAng(q, qd)
            
            varbls.mot.enviarPosArt(qt)

            q = varbls.sen.obtenerPosicion()
            
            q = np.radians(q)
            
            time.sleep(3)
            
            # Almacenamiento
            q_plot = np.zeros((2, N))
            trayctr_plot = np.zeros((2, N))
            qp_plot = np.zeros((2, N))
            e_plot = np.zeros((2, N))
            t_plot = np.zeros((1, N))

            # Configuración previa
            frecuencia_envio = 0.05 
            t0 = time.perf_counter()
            t_siguiente = t0
            
            while time.perf_counter() - t0 <= S:
                t_actual = time.perf_counter()
                
                if t_actual >= t_siguiente:
                    tk = t_actual - t0
                    print(f"{tk:.2f}")
                    
                    T1, T2, J = Jacobiano(q)
                    qkx, qpkx = evaluar_splines_c(Spx, t, tk)
                    qky, qpky = evaluar_splines_c(Spy, t, tk)
                    
                    xyd = np.asarray((qkx,qky))
                    xypd = np.asarray((qpkx,qpky))
                    
                    Ji = np.linalg.pinv(J[0:2,:])
                    e = xyd - T2
                    qp = Ji @ (xypd + D @ (e))
                    
                    varbls.mot.enviarVel((qp[0], qp[1]))
                    
                    q = varbls.sen.obtenerPosicion()
                    q = np.radians(q)
                    
                    # Guardar datos
                    t_plot[:,c] = tk
                    q_plot[:, c] = T2
                    trayctr_plot[:, c] = xyd
                    e_plot[:,c] = e
                    qp_plot[:,c] = qp
                    c += 1
                    
                    t_siguiente += frecuencia_envio 
            
            color = [1.0, 0.5, 0.0]
            print(c)
            plt.figure(num=1)
            plt.cla()
            
            plt.title("Trayectoria del robot")
            plt.xlabel("x (m)")
            plt.ylabel("y (m)")
            
            plt.plot(q_plot[0,:c], q_plot[1,:c], linewidth=2.5, color=color, linestyle='--', label="Robot")
            plt.plot(trayctr_plot[0,:c], trayctr_plot[1,:c], 'b', linewidth=1, label="Trayectoria")
            
            plt.grid()
            plt.legend()
            plt.axis('equal')
            plt.show()
            
            plt.figure(num=3)

            plt.title("Error")
            plt.xlabel("tiempo")

            plt.plot(t_plot[0,:c], e_plot[0,0:c],'r-', linewidth=1.5, label="ex")
            plt.plot(t_plot[0,:c], e_plot[1,:c],'b-', linewidth=1.5, label="ey")

            plt.grid()
            plt.legend()
            plt.show()

            # # %% Acción de control
            fig, axs = plt.subplots(2, 1, sharex=True)

            axs[0].plot(t_plot[0,:c], qp_plot[0,:c], 'r-', linewidth=1.5, label="M1")
            axs[0].set_ylabel("rad")
            axs[0].set_title("Acción de control")
            axs[0].grid()
            axs[0].legend()

            axs[1].plot(t_plot[0,:c], qp_plot[1,:c], 'r-', linewidth=1.5, label="M2")
            axs[1].set_ylabel("rad")
            axs[1].set_xlabel("tiempo [s]")
            axs[1].grid()
            axs[1].legend()

            plt.tight_layout()
            plt.show()
            
            varbls.mot.detenerMot()
            habilitarBtns()
            conectar.configure(state= 'enabled')
        
        # =============================================================================
        #   Botones  
        # =============================================================================
        
        conectar = ctk.CTkButton(frame_superior, text="Conectar", 
                                 font=('Segoe UI Semibold', 16),
                                 command=esperaConexion)
        
        conectar.pack(side="left", padx=15, pady=15)
        
        
        estado = ctk.CTkLabel(frame_superior, 
                              text="Robot Desconectado", 
                              text_color="#c62828", 
                              font=('Segoe UI Semibold', 16))
        
        estado.pack(side="left", padx=10, pady=15)
        
        # =============================================================================
        #  Frame Izquierdo    
        # =============================================================================
         
        carg = ctk.CTkButton(frame_izquierdo, 
                                text="Cargar Trayectoria", 
                                font=('Segoe UI Semibold', 14), 
                                command=esperarCargar)
    
        carg.pack(pady=[25,10], padx=20)
        
        carg.configure(state= 'disabled')
        
        ejecutar = ctk.CTkButton(frame_izquierdo, 
                                 text="Correr", 
                                 font=('Segoe UI Semibold', 16),
                                 command=esperarRobot)
        
        ejecutar.pack(pady=[25,10], padx=20)
        
        ejecutar.configure(state= 'disabled')
        
        mover = ctk.CTkButton(frame_mot_1,
                                 width=100,
                                 text="Mover a Pose", 
                                 font=('Segoe UI Semibold', 16),
                                 command=esperarMandarPos)
        
        mover.grid(row=0, column=1, pady=[10,30])
        
        mover.configure(state= 'disabled')
        
        izq_mot1 = ctk.CTkButton(frame_mot_1, 
                                 width=50,
                                 text="-", 
                                 font=('Segoe UI Semibold', 16),
                                 command=menosM1)
        
        izq_mot1.grid(row=1, column=0, sticky='e')
        
        izq_mot1.configure(state= 'disabled')
        
        # =============================================================================
        #   Frame Derecho      
        # =============================================================================
        
        der_mot1 = ctk.CTkButton(frame_mot_1,
                                 width=50,
                                 text="+", 
                                 font=('Segoe UI Semibold', 16),
                                 command=masM1)
        
        der_mot1.grid(row=1, column=2, sticky='w')
        
        der_mot1.configure(state= 'disabled')
        
        izq_mot2 = ctk.CTkButton(frame_mot_1, 
                                 width=50,
                                 text="-", 
                                 font=('Segoe UI Semibold', 16),
                                 command=menosM2)
        
        izq_mot2.grid(row=2, column=0, sticky='e')
        
        izq_mot2.configure(state= 'disabled')
        
        der_mot2 = ctk.CTkButton(frame_mot_1,
                                 width=50,
                                 text="+", 
                                 font=('Segoe UI Semibold', 16),
                                 command=masM2)
        
        der_mot2.grid(row=2, column=2, sticky='w')
        
        der_mot2.configure(state= 'disabled')
        
        carg_calib = ctk.CTkButton(frame_mot_1,
                                 text="Cargar Calib", 
                                 font=('Segoe UI Semibold', 16),
                                 command=cargarCalib)
        
        carg_calib.grid(row=4, column=1, pady=20)
        
        carg_calib.configure(state= 'disabled')
        
        calib = ctk.CTkButton(frame_mot_1,
                                 text="Calibrar", 
                                 font=('Segoe UI Semibold', 16),
                                 command=calibrar)
        
        calib.grid(row=5, column=1)
        
        calib.configure(state= 'disabled')
            
    # =============================================================================
    # PESTAÑA: Opciones
    # =============================================================================

    def crear_tab_op():
        
        tab_opciones = tabs.tab("Opciones")
  
        ctk.CTkLabel(tab_opciones, 
                     text="Configuración de Gráfica", 
                     font=("Arial Rounded MT Bold", 20)).pack(pady=20)
        
        frame_contenedor = ctk.CTkFrame(tab_opciones, fg_color="transparent")
        frame_contenedor.pack(pady=10) 
        
        frame_contenedor.grid_columnconfigure(0, minsize=100) 
        frame_contenedor.grid_columnconfigure(1, minsize=80)  
        frame_contenedor.grid_columnconfigure(2, minsize=80)  
        
        # =============================================================================
        # 1. LÍMITES X  
        # =============================================================================
        ctk.CTkLabel(frame_contenedor, text="Límites X:", font=('Segoe UI Semibold', 12)).grid(row=0, column=0, sticky="w", pady=8)
        
        limx0 = ctk.CTkEntry(frame_contenedor, placeholder_text="-35", width=70)
        limx0.insert(0, str(varbls.limx[0]))
        limx0.grid(row=0, column=1, sticky="w", pady=8)
        
        limx1 = ctk.CTkEntry(frame_contenedor, placeholder_text="35", width=70)
        limx1.insert(0, str(varbls.limx[1])) 
        limx1.grid(row=0, column=2, sticky="w", pady=8)
        
        # =============================================================================
        # 2. LÍMITES Y  
        # =============================================================================
        ctk.CTkLabel(frame_contenedor, text="Límites Y:", font=('Segoe UI Semibold', 12)).grid(row=1, column=0, sticky="w", pady=8)
        
        limy0 = ctk.CTkEntry(frame_contenedor, placeholder_text="-35", width=70)
        limy0.insert(0, str(varbls.limy[0]))
        limy0.grid(row=1, column=1, sticky="w", pady=8)
        
        limy1 = ctk.CTkEntry(frame_contenedor, placeholder_text="35", width=70)
        limy1.insert(0, str(varbls.limy[1]))
        limy1.grid(row=1, column=2, sticky="w", pady=8)
        
        # =============================================================================
        # 3. ESPACIADO  
        # =============================================================================        
        ctk.CTkLabel(frame_contenedor, text="Espaciado:", font=('Segoe UI Semibold', 12)).grid(row=2, column=0, sticky="w", pady=8)
        
        espcd = ctk.CTkEntry(frame_contenedor, placeholder_text="5", width=70)
        espcd.insert(0, str(varbls.punto_espcd[1]))
        espcd.grid(row=2, column=1, sticky="w", pady=8)
        
        # =============================================================================
        # 4. PUNTOS  
        # =============================================================================
        ctk.CTkLabel(frame_contenedor, text="T. Puntos:", font=('Segoe UI Semibold', 12)).grid(row=3, column=0, sticky="w", pady=8)
        
        punto = ctk.CTkEntry(frame_contenedor, placeholder_text="10", width=70)
        punto.insert(0, str(varbls.punto_espcd[0]))
        punto.grid(row=3, column=1, sticky="w", pady=8)
        
        activo2 = ctk.IntVar(value=True)
        switch = ctk.CTkSwitch(frame_contenedor, text='', variable=activo2, onvalue=1, offvalue=0)
        switch.grid(row=3, column=2, sticky="w", padx=5, pady=8)
        
        # =============================================================================
        # 5. LÍNEA GUÍA  
        # =============================================================================
        ctk.CTkLabel(frame_contenedor, text="Linea Guia:", font=('Segoe UI Semibold', 12)).grid(row=4, column=0, sticky="w", pady=8)
        
        switch2 = ctk.CTkSwitch(frame_contenedor, text='', variable=activo, onvalue=1, offvalue=0)
        switch2.grid(row=4, column=1, sticky="w", pady=8)
      
        # =============================================================================
        # FUNCIONES  
        # =============================================================================
        
        def actualizar_parametros_ejes():
            try:
                varbls.limx[0] = float(limx0.get())
                varbls.limx[1] = float(limx1.get())
                varbls.limy[0] = float(limy0.get())
                varbls.limy[1] = float(limy1.get())
                
                if activo2.get():
                    varbls.punto_espcd[0] = float(punto.get())
                else:
                    varbls.punto_espcd[0] = 0
                    
                varbls.punto_espcd[1] = float(espcd.get())
      
                actualizar_grafica()
                tabs.set("Gráfica") 
                
            except ValueError:
                messagebox.showerror("Error", "Por favor ingresa solo números válidos.")
      
        # =============================================================================
        # BOTÓN APLICAR  
        # =============================================================================
        btn_aplicar = ctk.CTkButton(tab_opciones, 
                                    text="Actualizar Gráfica", 
                                    command=actualizar_parametros_ejes)
        btn_aplicar.pack(pady=25)

    
    # =============================================================================
    #   Llamar para iniciar interfaz 
    # =============================================================================

    actualizar_grafica, cerrar_aplicacion = crear_tab_graf()
    crear_tab_simul()
    crear_tab_controls()
    crear_tab_op()
    
    raiz.protocol("WM_DELETE_WINDOW", cerrar_aplicacion)
    raiz.mainloop()
    
Interfaz()