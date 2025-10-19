import symforce
symforce.set_epsilon_to_number()

# symforce.set_symbolic_api("symengine")
symforce.set_symbolic_api("sympy")
symforce.set_log_level("warning")

import symforce.symbolic as sm
import numpy as np
from scipy.linalg import block_diag


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
def measurement_function(state_vec: sm.Matrix) -> sm.Matrix:
    """Funzione di misura: giroscopio e magnetometro."""
    q = state_vec[:4]
    omega = state_vec[4:7]
    
    # 1. Misurazione giroscopio (velocità angolare)
    gyro_meas = omega
    
    # 2. Misurazione magnetometro (campo magnetico nel body frame)
    mag_inertial = sm.Matrix([1, 0, 0])  # Campo magnetico inerziale (assunto lungo x)
    
    # Calcola la matrice di rotazione dal quaternione
    R = quaternion_to_rotation_matrix(q)
    mag_body = R * mag_inertial
    
    return gyro_meas.col_join(mag_body)
# Jacobiano simbolico della funzione di misura
H_expr = measurement_function(state).jacobian(state)

# ===========================================
# 4. Extended Kalman Filter Implementation
# ===========================================
class QuaternionEKF_RK4:
    def __init__(self, initial_state, initial_covariance):
        self.state = sm.Matrix(initial_state)  # Stato iniziale (7x1)
        self.covariance = sm.Matrix(initial_covariance)  # Covarianza iniziale (7x7)
        
        # Espressioni simboliche precalcolate
        self.F_expr = F_expr  # Jacobiano della transizione di stato
        self.H_expr = H_expr  # Jacobiano della funzione di misura
        
    def predict(self, dt_value: float, process_noise: sm.Matrix):
        """Passo di predizione dell'EKF."""
        # 1. Predizione dello stato
        self.state = state_transition_function(self.state, dt_value)
        
        # 2. Calcolo numerico dello Jacobiano F
        subs_dict = {state[i]: self.state[i] for i in range(7)}
        subs_dict[dt_sym] = dt_value
        F_num = self.F_expr.subs(subs_dict)
        
        # 3. Aggiornamento della covarianza
        self.covariance = F_num * self.covariance * F_num.T + process_noise
        
    def update(self, measurement: sm.Matrix, measurement_noise: sm.Matrix):
        """Passo di aggiornamento dell'EKF."""
        # 1. Predizione della misura
        z_pred = measurement_function(self.state)
        
        # 2. Calcolo numerico dello Jacobiano H
        subs_dict = {state[i]: self.state[i] for i in range(7)}
        H_num = self.H_expr.subs(subs_dict)
        
        # 3. Calcolo dell'innovazione
        y = measurement - z_pred
        
        # 4. Covarianza dell'innovazione
        S = H_num * self.covariance * H_num.T + measurement_noise
        
        # 5. Guadagno di Kalman
        K = self.covariance * H_num.T * S.inv()
        
        # 6. Correzione dello stato
        delta_x = K * y
        
        # Aggiornamento additivo dello stato
        self.state += delta_x
        
        # Normalizzazione del quaternione
        q_part = self.state[:4]
        norm_q = sm.sqrt(q_part[0]**2 + q_part[1]**2 + q_part[2]**2 + q_part[3]**2)
        self.state[0] /= norm_q
        self.state[1] /= norm_q
        self.state[2] /= norm_q
        self.state[3] /= norm_q
        
        # 7. Aggiornamento della covarianza (formula di Joseph)
        I = sm.Matrix.eye(7)
        I_KH = I - K * H_num
        self.covariance = I_KH * self.covariance * I_KH.T + K * measurement_noise * K.T

# ===========================================
# 5. Test completo del filtro
# ===========================================
if __name__ == "__main__":
    # Stato iniziale
    q0 = sm.Matrix([1, 0, 0, 0])
    omega0 = sm.Matrix([0.1, 0.2, 0.3])
    state0 = q0.col_join(omega0)
    
    # Inizializzazione EKF
    init_cov = sm.Matrix.eye(7) * 0.1
    ekf = QuaternionEKF_RK4(state0, init_cov)
    
    # Predizione
    dt_test = 0.05
    Q = sm.Matrix.diag([1e-6]*7)
    print("Stato prima della predizione:")
    print(state0)
    ekf.predict(dt_test, Q)
    print("\nStato dopo predizione:")
    print(ekf.state)
    
    # Misura simulata
    real_measurement = measurement_function(state0)
    measurement = real_measurement + sm.Matrix([0.01, -0.01, 0.02, 0.03, -0.02, 0.01])
    R = sm.Matrix.diag([0.1**2, 0.1**2, 0.1**2, 0.2**2, 0.2**2, 0.2**2])
    
    # Aggiornamento
    ekf.update(measurement, R)
    print("\nStato dopo aggiornamento:")
    print(ekf.state)
    
    # Seconda predizione
    ekf.predict(dt_test, Q)
    print("\nStato dopo seconda predizione:")
    print(ekf.state)
    
    # Verifica magnetometro
    print("\nMisura magnetometro attesa:", real_measurement[3:])
    print("Misura magnetometro simulata:", measurement[3:])
    
    # Verifica covarianza
    print("\nCovarianza finale:")
    print(np.array(ekf.covariance).astype(float))