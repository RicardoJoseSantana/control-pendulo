import numpy as np
import control as ct

# --- 1. PARÁMETROS DE TU SISTEMA (Ejemplo, reemplaza con los tuyos) ---
M = 0.320   # kg (masa del carro)
m = 0.020   # kg (masa del péndulo)
l = 0.13   # metros (longitud al centro de masa del péndulo)
g = 9.81  # m/s^2
I = (1/3)*m*(l**2) # kg*m^2 (momento de inercia, ej. (1/12)*m*(2*l)^2)

# --- 2. CONSTRUIR EL MODELO EN ESPACIO DE ESTADOS (A, B) ---
# x_dot = Ax + Bu
# Basado en la linealización de tus ecuaciones
den = I * (M + m) + M * m * l**2

A = np.array([
    [0, 1, 0, 0],
    [0, 0, (m**2 * g * l**2) / den, 0],
    [0, 0, 0, 1],
    [0, 0, (m * g * l * (M + m)) / den, 0]
])

B = np.array([
    [0],
    [(I + m * l**2) / den],
    [0],
    [(m * l) / den]
])

C = np.identity(4) # Matriz de salida (no la usaremos para LQR)
D = np.zeros((4,1))  # Matriz de transmisión directa (cero)

sys = ct.StateSpace(A, B, C, D)

# --- 3. DISEÑAR EL LQR (Matrices Q y R) ---
# Aquí es donde le dices al LQR qué es importante para ti.
# Q penaliza los errores de estado. R penaliza el esfuerzo de control.

# [Error de x, Error de x_dot, Error de theta, Error de theta_dot]
Q = np.diag([1.0, 1.0, 100.0, 1.0]) # Penalizamos MUCHO el error de ángulo (theta)

R = np.array([[0.1]]) # Penalización del esfuerzo de control (fuerza U)

# --- 4. CALCULAR LAS GANANCIAS K ---
K, S, E = ct.lqr(sys, Q, R)

print("Modelo del Sistema (Matriz A):\n", A)
print("\nModelo del Sistema (Matriz B):\n", B)
print("\n----------------------------------------------------")
print("GANANCIAS DEL CONTROLADOR LQR (K):")
print(f"k1 (x)     = {K[0,0]}")
print(f"k2 (x_dot) = {K[0,1]}")
print(f"k3 (theta) = {K[0,2]}")
print(f"k4 (theta_dot) = {K[0,3]}")
print("----------------------------------------------------")
print("\n¡Copia estas 4 constantes en tu archivo pid_controller.c!")