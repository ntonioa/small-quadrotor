import symforce
symforce.set_epsilon_to_number()
symforce.set_symbolic_api("symengine")

import symforce.symbolic as sm
from symforce.codegen import Codegen
from symforce.codegen.backends.cpp.cpp_config import CppConfig

# -------------------------------------------------------
# Inputs simbolici condivisi
# -------------------------------------------------------
# Stato attuale (EKF)
q = sm.Quaternion.symbolic("q")        # [w,x,y,z]
omega = sm.Vector3.symbolic("omega")   # rad/s (body)

# Gain
kR = sm.Vector3.symbolic("kR")         # [kR_x, kR_y, kR_z]
kW = sm.Vector3.symbolic("kW")         # [kW_x, kW_y, kW_z]

# Inerzia completa
Jxx,Jyy,Jzz = sm.Symbol("Jxx"), sm.Symbol("Jyy"), sm.Symbol("Jzz")
Jxy,Jxz,Jyz = sm.Symbol("Jxy"), sm.Symbol("Jxz"), sm.Symbol("Jyz")
J = sm.Matrix(3,3,[Jxx,Jxy,Jxz,  Jxy,Jyy,Jyz,  Jxz,Jyz,Jzz])

# Comandi “pilota” (telefono)
phi_d   = sm.Symbol("phi_d")           # roll desired [rad]
theta_d = sm.Symbol("theta_d")         # pitch desired [rad]
psi_d   = sm.Symbol("psi_d")           # yaw reference [rad]
throttle = sm.Symbol("throttle")       # throttle [0..1]

# Parametri massa/gravità
m = sm.Symbol("m")
g = sm.Symbol("g")

# -------------------------------------------------------
# Utility
# -------------------------------------------------------
def quat_error_vec(qd: sm.Quaternion, q: sm.Quaternion) -> sm.Vector3:
    # q_err = qd^{-1} ⊗ q ; eR = 2*sign(w)*vec
    qe = qd.inverse() * q
    return 2 * sm.sign(qe.w) * sm.Vector3(qe.x, qe.y, qe.z)

def q_from_rpy_storage(phi, theta, psi):
    # half-angles
    hphi   = phi   / 2
    htheta = theta / 2
    hpsi   = psi   / 2

    c1, s1 = sm.cos(hpsi),   sm.sin(hpsi)     # yaw/2
    c2, s2 = sm.cos(htheta), sm.sin(htheta)   # pitch/2
    c3, s3 = sm.cos(hphi),   sm.sin(hphi)     # roll/2

    # Rz(psi)*Ry(theta)*Rx(phi) -> quaternion [w,x,y,z]
    qw = c1*c2*c3 + s1*s2*s3
    qx = s3*c2*c1 - c3*s2*s1
    qy = c3*s2*c1 + s3*c2*s1
    qz = c3*c2*s1 - s3*s2*c1

    return sm.Matrix([qw, qx, qy, qz])  # storage [w,x,y,z]


def tau_att(q: sm.Quaternion, qd: sm.Quaternion,
            omega: sm.Vector3, kR: sm.Vector3, kW: sm.Vector3, J: sm.Matrix) -> sm.Vector3:
    # errore assetto (quaternion geometric-like)
    qe = qd.inverse() * q
    eR = 2 * sm.sign(qe.w) * sm.Vector3(qe.x, qe.y, qe.z)

    # errore velocità (omega_d = 0 per iniziare)
    eW = omega

    # Coriolis: ω × (J ω)
    Jw = J * omega
    coriolis = omega.cross(Jw)

    # "diag(kR)*eR" e "diag(kW)*eW" fatti elemento-per-elemento
    kR_eR = sm.Vector3(kR[0]*eR[0], kR[1]*eR[1], kR[2]*eR[2])
    kW_eW = sm.Vector3(kW[0]*eW[0], kW[1]*eW[1], kW[2]*eW[2])

    tau = -kR_eR - kW_eW + coriolis
    return tau


def thrust_from_throttle(throttle, m, g):
    # Modello semplice: Fz = m*g*throttle (1.0 ≈ hover)
    return sm.Matrix([m*g*throttle])  # 1x1

def make_wrench(tau: sm.Vector3, Fz: sm.Matrix):
    # y = [tau_x, tau_y, tau_z, Fz]
    return sm.Vector4(tau[0], tau[1], tau[2], Fz[0])

# -------------------------------------------------------
# 1) qd_from_rpy: (phi_d, theta_d, psi_d) -> qd_storage [w,x,y,z]
# -------------------------------------------------------
Codegen.function(
    func=lambda phi,theta,psi: sm.Matrix(q_from_rpy_storage(phi,theta,psi)),
    input_types=[phi_d, theta_d, psi_d],
    output_names=["qd_storage"],
    name="qd_from_rpy",
    config=CppConfig(),
).generate_function("generated")

# -------------------------------------------------------
# 2) geo_attitude_only: (q, qd, omega, kR, kW, J) -> tau (Vector3)
# -------------------------------------------------------
qd_sym = sm.Quaternion.symbolic("qd")
Codegen.function(
    func=lambda q,qd,omega,kR,kW,J: sm.Matrix(tau_att(q,qd,omega,kR,kW,J)),
    input_types=[q, qd_sym, omega, kR, kW, J],
    output_names=["tau"],
    name="geo_attitude_only",
    config=CppConfig(),
).generate_function("generated")

# -------------------------------------------------------
# 3) thrust_from_throttle: (throttle, m, g) -> Fz (Scalar 1x1)
# -------------------------------------------------------
Codegen.function(
    func=lambda throttle,m,g: thrust_from_throttle(throttle,m,g),
    input_types=[throttle, m, g],
    output_names=["Fz"],
    name="thrust_from_throttle",
    config=CppConfig(),
).generate_function("generated")

# -------------------------------------------------------
# 4) make_wrench: (tau, Fz) -> y (Vector4)
# -------------------------------------------------------
tau_in = sm.Vector3.symbolic("tau_in")
Fz_in  = sm.Matrix(1,1,[sm.Symbol("Fz_in")])
Codegen.function(
    func=lambda tau, Fz: sm.Matrix(make_wrench(tau, Fz)),
    input_types=[tau_in, Fz_in],
    output_names=["y"],
    name="make_wrench",
    config=CppConfig(),
).generate_function("generated")

# -------------------------------------------------------
# 5) (opzionale) step unico: input comandi -> y
#    q, omega, phi_d, theta_d, psi_d, kR, kW, J, m, g, throttle  -> y (Vector4)
# -------------------------------------------------------
def attitude_controller_step(q, omega, phi, theta, psi, kR, kW, J, m, g, throttle):
    qd_storage = q_from_rpy_storage(phi, theta, psi)
    qd = sm.Quaternion.from_storage(qd_storage)
    tau = tau_att(q, qd, omega, kR, kW, J)
    Fz  = thrust_from_throttle(throttle, m, g)
    return make_wrench(tau, Fz)

Codegen.function(
    func=lambda q,omega,phi,theta,psi,kR,kW,J,m,g,thr: sm.Matrix(
        attitude_controller_step(q,omega,phi,theta,psi,kR,kW,J,m,g,thr)
    ),
    input_types=[q, omega, phi_d, theta_d, psi_d, kR, kW, J, m, g, throttle],
    output_names=["y"],
    name="attitude_controller_step",
    config=CppConfig(),
).generate_function("generated")

if __name__ == "__main__":
    print("C++ code generated in './generated/'")
