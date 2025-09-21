import tkinter as tk
from tkinter import ttk
import serial
import time
import threading

# --- CONFIGURACIÓN ---
SERIAL_PORT = 'COM8'  # ¡Asegúrate de que este es el puerto correcto!
BAUD_RATE = 115200
# ---------------------

class PidTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Sintonizador PID para ESP32")
        
        self.ser = None
        self.connect_serial()

        # --- Crear los widgets de la GUI ---
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # --- Sliders para Kp, Ki, Kd ---
        self.kp_val = tk.DoubleVar(value=1.0)
        self.ki_val = tk.DoubleVar(value=0.0)
        self.kd_val = tk.DoubleVar(value=0.0)
        
        self.create_slider("Kp (Proporcional):", self.kp_val, 0, 10, self.send_kp)
        self.create_slider("Ki (Integral):", self.ki_val, 0, 5, self.send_ki)
        self.create_slider("Kd (Derivativo):", self.kd_val, 0, 1, self.send_kd)

        # --- Consola de Salida ---
        ttk.Label(main_frame, text="Salida del ESP32:").grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(10,0))
        self.output_text = tk.Text(main_frame, height=10, width=60)
        self.output_text.grid(row=4, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        # Iniciar un hilo para leer del puerto serie sin bloquear la GUI
        self.is_running = True
        self.thread = threading.Thread(target=self.read_from_port)
        self.thread.daemon = True
        self.thread.start()

        # Configurar el cierre de la ventana
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Conectado a {SERIAL_PORT} a {BAUD_RATE} baudios.")
        except serial.SerialException as e:
            print(f"Error al conectar al puerto {SERIAL_PORT}: {e}")
            # Crear una ventana de error
            error_win = tk.Toplevel(self.root)
            error_win.title("Error de Conexión")
            ttk.Label(error_win, text=f"No se pudo conectar a {SERIAL_PORT}.\nVerifica que el puerto es correcto y no está en uso.").pack(padx=20, pady=20)
            ttk.Button(error_win, text="OK", command=error_win.destroy).pack(pady=10)
            self.ser = None

    def create_slider(self, label_text, variable, from_, to, command):
        frame = ttk.Frame(self.root.winfo_children()[0]) # Asegurarse de que se añade al main_frame
        frame.grid(sticky=(tk.W, tk.E), pady=5)
        
        label = ttk.Label(frame, text=label_text, width=20)
        label.pack(side=tk.LEFT)
        
        slider = ttk.Scale(frame, from_=from_, to=to, orient=tk.HORIZONTAL, variable=variable, command=command, length=300)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        value_label = ttk.Label(frame, textvariable=variable, width=10)
        variable.trace_add("write", lambda *args: value_label.config(text=f"{variable.get():.2f}"))
        value_label.pack(side=tk.LEFT, padx=5)
        value_label.config(text=f"{variable.get():.2f}")


    def send_command(self, command_string):
        if self.ser and self.ser.is_open:
            print(f"Enviando: {command_string}")
            self.ser.write(command_string.encode('utf-8'))
        else:
            print("No conectado. Comando no enviado.")

    def send_kp(self, value):
        self.send_command(f"SETKP {float(value):.4f}\n")

    def send_ki(self, value):
        self.send_command(f"SETKI {float(value):.4f}\n")

    def send_kd(self, value):
        self.send_command(f"SETKD {float(value):.4f}\n")
        
    def read_from_port(self):
        while self.is_running and self.ser:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    # Usamos 'schedule' para actualizar la GUI desde este hilo de forma segura
                    self.root.after(0, self.update_text, line)
            except (serial.SerialException, UnicodeDecodeError):
                # El puerto se desconectó o se recibieron datos corruptos
                self.root.after(0, self.update_text, "--- Puerto Desconectado ---")
                break

    def update_text(self, text):
        self.output_text.insert(tk.END, text + '\n')
        self.output_text.see(tk.END) # Auto-scroll hacia el final

    def on_closing(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PidTunerApp(root)
    root.mainloop()