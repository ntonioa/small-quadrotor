import symforce
symforce.set_epsilon_to_number()
symforce.set_symbolic_api("symengine")

import symforce.symbolic as sm
import numpy as np
from scipy.linalg import block_diag

# Code generation imports
from symforce.codegen import Codegen
from symforce.codegen.backends.cpp.cpp_config import CppConfig
from symforce import typing as T

# ===========================================
# 1. Symbolic Variables and State Definition
# ===========================================
q_w = sm.Symbol("q_w")
q_x = sm.Symbol("q_x")
q_y = sm.Symbol("q_y")
q_z = sm.Symbol("q_z")

wx = sm.Symbol("wx")
wy = sm.Symbol("wy")
wz = sm.Symbol("wz")

dt = sm.Symbol("dt")

# Define the full state vector
state = sm.Matrix([q_w, q_x, q_y, q_z, wx, wy, wz])
dt_sym = sm.Symbol("dt_sym")  # Symbolic dt for Jacobian computation

def quaternion_derivative(q: sm.Matrix, omega: sm.Matrix) -> sm.Matrix:
    q_sf = sm.Quaternion.from_storage(q)
    omega_quat = sm.Quaternion.from_storage(sm.Matrix([0, omega[0], omega[1], omega[2]]))
    # composizione quaternionica
    q_dot_quat = q_sf * omega_quat
    # estrai lo storage e crea un Matrix, poi scala
    storage = q_dot_quat.to_storage()          # tuple([ẇ, ẋ, ẏ, ż])
    q_dot = sm.Matrix(storage) * sm.S(0.5)     # ora è un Matrix 4×1
    return q_dot


def rk4_integration(q: sm.Matrix, omega: sm.Matrix, dt: float) -> sm.Matrix:
    k1 = quaternion_derivative(q, omega)
    k2 = quaternion_derivative(q + (dt/2) * k1, omega)
    k3 = quaternion_derivative(q + (dt/2) * k2, omega)
    k4 = quaternion_derivative(q + dt * k3, omega)

    q_new = q + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    # calcolo manuale della norma
    norm_sq = q_new[0]**2 + q_new[1]**2 + q_new[2]**2 + q_new[3]**2
    norm_q = sm.sqrt(norm_sq)

    # normalizzo
    q_new_norm = q_new / norm_q
    return q_new_norm

def state_transition_function(state_vec: sm.Matrix, dt: float) -> sm.Matrix:
    # Split
    q = state_vec[:4]
    omega = state_vec[4:]

    # Integrazione quaternione
    q_new = rk4_integration(q, omega, dt)

    # Ricostruisci lo stato concatenando liste
    elems = list(q_new) + list(omega)  # prima 4 di q_new, poi 3 di omega
    return sm.Matrix(elems)            # Matrix[7×1]


dt_sym = sm.Symbol("dt_sym")
state = sm.Matrix([q_w, q_x, q_y, q_z, wx, wy, wz])
F_expr = state_transition_function(state, dt_sym).jacobian(state)

# ===========================================
# 3. Measurement Function and Jacobian
# ===========================================
def quaternion_to_rotation_matrix(q: sm.Matrix) -> sm.Matrix:
    """Converte un quaternione in matrice di rotazione 3x3."""
    w, x, y, z = q
    return sm.Matrix([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])
# def measurement_function(state_vec: sm.Matrix) -> sm.Matrix:
#     """Funzione di misura: giroscopio e magnetometro."""
#     q = state_vec[:4]
#     omega = state_vec[4:7]
    
#     # 1. Misurazione giroscopio (velocità angolare)
#     gyro_meas = omega
    
#     # 2. Misurazione magnetometro (campo magnetico nel body frame)
#     mag_inertial = sm.Matrix([1, 0, 0])  # Campo magnetico inerziale (assunto lungo x)
    
#     # Calcola la matrice di rotazione dal quaternione
#     R = quaternion_to_rotation_matrix(q)
#     mag_body = R * mag_inertial
    
#     return gyro_meas.col_join(mag_body)

def measurement_function(state_vec: sm.Matrix) -> sm.Matrix:
    q = state_vec[:4]
    # Accelerazione prevista = gravità ruotata in body frame
    g_world = sm.Matrix([0, 0, -1])  # o -9.81
    R = quaternion_to_rotation_matrix(q)
    acc_pred = R * g_world
    return acc_pred

# Jacobiano simbolico della funzione di misura
H_expr = measurement_function(state).jacobian(state)


# ===========================================
# 4. Predict and Update Functions for Codegen
# ===========================================
def predict_sym(state: sm.V7, dt: sm.Scalar, covariance: sm.Matrix77, process_noise: sm.Matrix77) -> T.Tuple[sm.V7, sm.Matrix77]:
    """Symbolic prediction function."""
    new_state = state_transition_function(state, dt)
    # Substitute state symbols with input state vector elements
    subs_dict = {q_w: state[0], q_x: state[1], q_y: state[2], q_z: state[3], 
                 wx: state[4], wy: state[5], wz: state[6], dt_sym: dt}
    F_num = F_expr.subs(subs_dict)
    new_covariance = F_num * covariance * F_num.T + process_noise
    return new_state, new_covariance

def update_sym(state: sm.V7, covariance: sm.Matrix77, measurement: sm.V6, measurement_noise: sm.Matrix66) -> T.Tuple[sm.V7, sm.Matrix77]:
    """Symbolic update function."""
    z_pred = measurement_function(state)
    subs_dict = {q_w: state[0], q_x: state[1], q_y: state[2], q_z: state[3], 
                 wx: state[4], wy: state[5], wz: state[6]}
    H_num = H_expr.subs(subs_dict)
    y = measurement - z_pred
    S = H_num * covariance * H_num.T + measurement_noise
    K = covariance * H_num.T * S.inv()
    delta_x = K * y
    new_state = state + delta_x
    q_part = new_state[:4]
    norm_q = sm.sqrt(q_part[0]**2 + q_part[1]**2 + q_part[2]**2 + q_part[3]**2)
    new_state[0] = new_state[0] / norm_q
    new_state[1] = new_state[1] / norm_q
    new_state[2] = new_state[2] / norm_q
    new_state[3] = new_state[3] / norm_q
    I = sm.Matrix77.eye()
    I_KH = I - K * H_num
    new_covariance = I_KH * covariance * I_KH.T + K * measurement_noise * K.T
    return new_state, new_covariance



## ===========================================
# Codegen for ESP32 (C++)
# ===========================================

def generate_cpp(
    scalar_type="float",       
    optimize=True,
    include_metadata=False,
    namespace="ekf",      
):
    # Predict state
    Codegen.function(
        func=state_transition_function,
        input_types=[state, dt_sym],
        output_names=["x_next"],
        name="predict_state",
        config=CppConfig(),
    ).generate_function("generated")

    # Jacobian of F
    Codegen.function(
        func=lambda x, dt: state_transition_function(x, dt).jacobian(x),
        input_types=[state, dt_sym],
        output_names=["F"],
        name="jacobian_F",
        config=CppConfig(),
    ).generate_function("generated")

    # Predict measurement
    Codegen.function(
        func=measurement_function,
        input_types=[state],
        output_names=["z_pred"],
        name="predict_measurement",
        config=CppConfig(),
    ).generate_function("generated")

    # Jacobian of H
    Codegen.function(
        func=lambda x: measurement_function(x).jacobian(x),
        input_types=[state],
        output_names=["H"],
        name="jacobian_H",
        config=CppConfig(),
    ).generate_function("generated")

# Predict function
    Codegen.function(
        func=predict_sym,
        input_types=[sm.V7, sm.Scalar, sm.Matrix77, sm.Matrix77],
        output_names=["new_state", "new_covariance"],
        name="predict",
        config=CppConfig(),
    ).generate_function("generated_ekf")

    # Update function
    Codegen.function(
        func=update_sym,
        input_types=[sm.V7, sm.Matrix77, sm.V6, sm.Matrix66],
        output_names=["new_state", "new_covariance"],
        name="update",
        config=CppConfig(),
    ).generate_function("generated_ekf")


if __name__ == "__main__":
    generate_cpp()
    print("C++ code generated in './generated/'")
