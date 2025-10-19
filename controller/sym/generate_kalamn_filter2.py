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

def state_transition_function(state_vec: sm.Matrix, dt: float) -> sm.Matrix:
    q = state_vec[:4]
    omega = state_vec[4:]
    q_new = rk4_integration(q, omega, dt)
    return sm.Matrix(list(q_new) + list(omega))

F_expr = state_transition_function(state, dt_sym).jacobian(state)

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
    g_world = sm.Matrix([0, 0, -1])  # puoi mettere -9.81 se vuoi valori in m/s^2
    R = quaternion_to_rotation_matrix(q)
    acc_pred = R * g_world
    return acc_pred   # 3x1

H_expr = measurement_function(state).jacobian(state)

def predict_sym(state: sm.V7, dt: sm.Scalar, covariance: sm.Matrix77, process_noise: sm.Matrix77) -> T.Tuple[sm.V7, sm.Matrix77]:
    new_state = state_transition_function(state, dt)
    subs_dict = {q_w: state[0], q_x: state[1], q_y: state[2], q_z: state[3],
                 wx: state[4], wy: state[5], wz: state[6], dt_sym: dt}
    F_num = F_expr.subs(subs_dict)
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
        input_types=[sm.V7, sm.Matrix77, sm.V3, sm.Matrix33],  # <-- SOLO ACC!
        output_names=["new_state", "new_covariance"],
        name="update",
        config=CppConfig(),
    ).generate_function("generated_ekf")

if __name__ == "__main__":
    generate_cpp()
    print("C++ code generated in './generated/'")
