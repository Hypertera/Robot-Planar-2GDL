import tkinter as tk
from tkinter import messagebox, filedialog
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.ticker import MultipleLocator

class AppPlano:
    def __init__(self, root):
        
        self.Robot = 30
        self.base = 6
        
        self.t = np.linspace(0, 2*np.pi, 500)
        
        self.rx = self.Robot*np.cos(self.t)
        self.ry = self.Robot*np.sin(self.t)
        
        self.rbx = self.base*np.cos(self.t)
        self.rby = self.base*np.sin(self.t)
        
        self.bdt = 0.02
        self.S = 1
        
        self.limx = [-35,35]
        self.limy = [-35,35]
        self.espcd = 5
        
        self.root = root
        self.root.title("Curvas de Bézier Continuas")
        
        self.root.protocol("WM_DELETE_WINDOW", self.cerrar_aplicacion)

        self.puntos_x = []
        self.puntos_y = []
        self.bezier = None

        # Interfaz
        frame_input = tk.Frame(root)
        frame_input.pack(pady=10)

        tk.Label(frame_input, text="X:").pack(side=tk.LEFT)
        self.entry_x = tk.Entry(frame_input, width=5)
        self.entry_x.pack(side=tk.LEFT, padx=5)

        tk.Label(frame_input, text="Y:").pack(side=tk.LEFT)
        self.entry_y = tk.Entry(frame_input, width=5)
        self.entry_y.pack(side=tk.LEFT, padx=5)

        tk.Button(frame_input, text="Añadir Punto", 
                  command=self.agregar_manual).pack(side=tk.LEFT, padx=5)
        
        tk.Button(frame_input, text="Guardar", 
                  command=self.guardar).pack(side=tk.LEFT, padx=5)
        
        tk.Button(frame_input, text="Remover Punto", 
                  command=self.borrar_punto, fg="red").pack(side=tk.LEFT, padx=5)
        
        tk.Button(frame_input, text="Limpiar", 
                  command=self.limpiar, fg="red").pack(side=tk.LEFT, padx=5)

        # Gráfica
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.configurar_ejes()

        self.canvas_matplotlib = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas_widget = self.canvas_matplotlib.get_tk_widget()
        self.canvas_widget.pack(fill=tk.BOTH, expand=True)

        self.fig.canvas.mpl_connect('button_press_event', self.al_hacer_clic)

    def configurar_ejes(self):
        self.ax.clear()
        self.ax.axhline(0, color='black', linewidth=1.2)
        self.ax.axvline(0, color='black', linewidth=1.2)
        self.ax.set_xlabel("X (cm)", fontsize=10)
        self.ax.set_ylabel("Y (cm)", fontsize=10)
        self.ax.set_xlim(self.limx[0], self.limx[1])
        self.ax.set_ylim(self.limy[0], self.limy[1])
        self.ax.xaxis.set_major_locator(MultipleLocator(self.espcd))
        self.ax.yaxis.set_major_locator(MultipleLocator(self.espcd))
        self.ax.grid(True, linestyle='-', alpha=0.9)
        self.ax.set_title("Haz clic para añadir puntos")
        self.ax.plot(self.rx, self.ry, 'r--', label='Area de trabajo')
        self.ax.plot(self.rbx, self.rby, 'r--')
        self.ax.legend()

    def calcular_Trayectoria(self, P0, P1, P2, P3, bdt, S, i, xy):
        M = np.array([
            [-1,  3, -3,  1],
            [ 3, -6,  3,  0],
            [-3,  3,  0,  0],
            [ 1,  0,  0,  0]
        ])
        P = np.column_stack((P0, P1, P2, P3))
        tiempos = np.arange(0, S + (bdt / 10), bdt)
        
        for t in tiempos:
            tn = t / S
            T = np.array([tn**3, tn**2, tn, 1])
            pts = P @ M @ T
            xy[:, i] = pts
            i += 1
        return i, xy

    def actualizar_grafica(self):
        self.configurar_ejes()
        
        self.trayectoria_completa = []
        
        self.ax.scatter(self.puntos_x, self.puntos_y, color='black', s=30, zorder=5)
        
        if len(self.puntos_x) > 1:
            self.ax.plot(self.puntos_x, self.puntos_y, 'k--', alpha=0.8)

        num_puntos = len(self.puntos_x)
        if num_puntos >= 4:
            for start_idx in range(0, num_puntos - 3, 3):
                p_indices = range(start_idx, start_idx + 4)
                
                P0 = [self.puntos_x[p_indices[0]], self.puntos_y[p_indices[0]]]
                P1 = [self.puntos_x[p_indices[1]], self.puntos_y[p_indices[1]]]
                P2 = [self.puntos_x[p_indices[2]], self.puntos_y[p_indices[2]]]
                P3 = [self.puntos_x[p_indices[3]], self.puntos_y[p_indices[3]]]

                tiempos = np.arange(0, self.S + (self.bdt / 10), self.bdt)
                xy = np.zeros((2, len(tiempos)))
                
                _, trayectoria = self.calcular_Trayectoria(P0, P1, P2, P3, 
                                                           self.bdt, self.S, 0, xy)
                
                self.trayectoria_completa.append(trayectoria)
                
                self.bezier = np.hstack(self.trayectoria_completa) 
                
                self.ax.plot(trayectoria[0, :], trayectoria[1, :], 
                             color='blue', linewidth=2)

        else:
            self.bezier = None
            
        self.canvas_matplotlib.draw()

    def al_hacer_clic(self, event):
        if event.inaxes:
            self.puntos_x.append(round(event.xdata, 2))
            self.puntos_y.append(round(event.ydata, 2))
            self.actualizar_grafica()

    def agregar_manual(self):
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            self.puntos_x.append(x)
            self.puntos_y.append(y)
            self.actualizar_grafica()
        except ValueError:
            messagebox.showerror("Error", "Dato inválido")
            
    def guardar(self):
        if self.bezier is not None:
            ruta_archivo = filedialog.asksaveasfilename(
                defaultextension=".npy",
                filetypes=[("Archivo NumPy", "*.npy"), ("Todos los archivos", "*.*")],
                title="Guardar trayectoria como..."
            )
            
            if ruta_archivo:
                np.save(ruta_archivo, self.bezier)
                messagebox.showinfo("Éxito", f"Trayectoria guardada en:\n{ruta_archivo}")
        else:
            messagebox.showwarning("Aviso", "No hay trayectoria generada para guardar")

    def borrar_punto(self):
        if len(self.puntos_x) > 0:
            self.puntos_x.pop()
            self.puntos_y.pop()
            self.actualizar_grafica()
        else:
            messagebox.showwarning("Aviso", "No hay más puntos para eliminar")
    
    def limpiar(self):
        if len(self.puntos_x) > 0:
            self.puntos_x, self.puntos_y = [], []
            self.actualizar_grafica()
        else:
            messagebox.showwarning("Aviso", "No hay más puntos para eliminar")   
        
    def cerrar_aplicacion(self):
        self.root.quit()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = AppPlano(root)
    root.mainloop()
