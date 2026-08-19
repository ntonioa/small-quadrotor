import symforce
symforce.set_epsilon_to_number()
symforce.set_symbolic_api("symengine")

import symforce.symbolic as sm
from symforce.codegen import Codegen
from symforce.codegen.backends.cpp.cpp_config import CppConfig
from symforce import typing as T

# Stato: quaternion + gyro bias (7x1)
q_w, q_x, q_y, q_z = sm.Symbol("q_w"), sm.Symbol("q_x"), sm.Symbol("q_y"), sm.Symbol("q_z")
wx, wy, wz = sm.Symbol("wx"), sm.Symbol("wy"), sm.Symbol("wz")
dt_sym = sm.Symbol("dt_sym")
state = sm.Matrix([q_w, q_x, q_y, q_z, wx, wy, wz])

wx_m, wy_m, wz_m = sm.Symbol("wx_m"), sm.Symbol("wy_m"), sm.Symbol("wz_m")
omega_measured_sym = sm.Matrix([wx_m, wy_m, wz_m])

def quaternion_derivative(q: sm.Matrix, omega: sm.Matrix) -> sm.Matrix:
    q_sf = sm.Quaternion.from_storage(q)
    omega_quat = sm.Quaternion.from_storage(sm.Matrix([0, omega[0], omega[1], omega[2]]))
    q_dot_quat = q_sf * omega_quat
    q_dot = sm.Matrix(q_dot_quat.to_storage()) * sm.S(0.5)
    return q_dot

def rk4_integration(q: sm.Matrix, omega: sm.Matrix, dt: float) -> sm.Matrix:
    k1 = quaternion_derivative(q, omega)
    k2 = quaternion_derivative(q + (dt/2) * k1, omega)
    k3 = quaternion_derivative(q + (dt/2) * k2, omega)
    k4 = quaternion_derivative(q + dt * k3, omega)
    q_new = q + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
    norm_q = sm.sqrt(sum([q_new[i]**2 for i in range(4)]))
    return q_new / norm_q


def state_transition_function(state_vec: sm.Matrix, omega_measured: sm.Matrix, dt: float) -> sm.Matrix:
    q = state_vec[:4]
    bias = state_vec[4:]
    
    # Logica corretta: la velocità angolare è (misura - bias)
    omega_corrected = omega_measured - bias
    
    # Propaga il quaternione con la velocità corretta
    q_new = rk4_integration(q, omega_corrected, dt)
    
    # Il bias rimane costante (il rumore di processo Q lo aggiornerà)
    return sm.Matrix(list(q_new) + list(bias))

F_expr = state_transition_function(state, omega_measured_sym, dt_sym).jacobian(state) 


def quaternion_to_rotation_matrix(q: sm.Matrix) -> sm.Matrix:
    w, x, y, z = q
    return sm.Matrix([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])

def measurement_function(state_vec: sm.Matrix) -> sm.Matrix:
    q = state_vec[:4]
    # Accelerazione prevista = gravità ruotata nel body
    g_world = sm.Matrix([0, 0, -9.81]) 
    R = quaternion_to_rotation_matrix(q)
    acc_pred = R.T * g_world
    return acc_pred   # 3x1

H_expr = measurement_function(state).jacobian(state)

def predict_sym(state: sm.V7, omega_measured: sm.V3, dt: sm.Scalar, covariance: sm.Matrix77, process_noise: sm.Matrix77) -> T.Tuple[sm.V7, sm.Matrix77]:
    
    # 1. Predici il nuovo stato usando la misura del giroscopio
    new_state = state_transition_function(state, omega_measured, dt)
    
    # 2. Sostituisci TUTTI i valori per calcolare F numerico
    #    (wx, wy, wz sono i BIAS dallo stato)
    subs_dict = {q_w: state[0], q_x: state[1], q_y: state[2], q_z: state[3],
                 wx: state[4], wy: state[5], wz: state[6],
                 wx_m: omega_measured[0], wy_m: omega_measured[1], wz_m: omega_measured[2],
                 dt_sym: dt}
    F_num = F_expr.subs(subs_dict)
    
    # 3. Propaga la covarianza
    new_covariance = F_num * covariance * F_num.T + process_noise
    return new_state, new_covariance

def update_sym(state: sm.V7, covariance: sm.Matrix77, measurement: sm.V3, measurement_noise: sm.Matrix33) -> T.Tuple[sm.V7, sm.Matrix77]:
    z_pred = measurement_function(state)  # 3x1
    subs_dict = {q_w: state[0], q_x: state[1], q_y: state[2], q_z: state[3],
                 wx: state[4], wy: state[5], wz: state[6]}
    H_num = H_expr.subs(subs_dict)  # 3x7
    y = measurement - z_pred        # 3x1
    S = H_num * covariance * H_num.T + measurement_noise  # 3x3
    K = covariance * H_num.T * S.inv()                    # 7x3
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


def generate_cpp(
    scalar_type="float",
    optimize=True,
    include_metadata=False,
    namespace="ekf",
):

    # Predict function
    Codegen.function(
        func=predict_sym,
        input_types=[sm.V7, sm.V3, sm.Scalar, sm.Matrix77, sm.Matrix77],
        output_names=["new_state", "new_covariance"],
        name="predict",
        config=CppConfig(),
    ).generate_function("generated_ekf")

    # Update function
    Codegen.function(
        func=update_sym,
        input_types=[sm.V7, sm.Matrix77, sm.V3, sm.Matrix33],  # <-- SOLO ACC!
        output_names=["new_state", "new_covariance"],
        name="update",
        config=CppConfig(),
    ).generate_function("generated_ekf")

if __name__ == "__main__":
    generate_cpp()
    print("C++ code generated in './generated/'")
