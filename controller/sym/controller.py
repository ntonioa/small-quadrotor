import symforce
symforce.set_symbolic_api("symengine")
symforce.set_epsilon_to_symbol()
import symforce.symbolic as sm
from symforce.codegen import Codegen

# ---- Inputs ----
# target wrench
y = sm.Vector4.symbolic("y")   # [tau_x, tau_y, tau_z, Fz]

# rotor geometry (N=4 esempio; estendi a N)
x = sm.Vector4.symbolic("x_pos")
yy = sm.Vector4.symbolic("y_pos")
s = sm.Vector4.symbolic("spin")   # +1/-1
c = sm.Vector4.symbolic("c_drag") # c_i = k_m/k_f (o equivalente)

# Motor map params (quadratic with deadzone): f_i = k_i * max(0, u_i - u0_i)^2
k  = sm.Vector4.symbolic("k_thrust")
u0 = sm.Vector4.symbolic("u0_deadzone")
u_min = sm.Vector4.symbolic("u_min")
u_max = sm.Vector4.symbolic("u_max")

# Optional thrust bounds (derivano da u_min/u_max)
# fmin_i = k_i * max(0, u_min_i - u0_i)^2; idem per fmax
def quad_inv(f, k_i, u0_i):
    # inverse of f = k*(u-u0)^2, assuming f >= 0
    return u0_i + sm.sqrt(sm.max(0, f / k_i))

# ---- Build allocation matrix B (4x4 for a quad) ----
def build_B(x, yy, s, c):
    # columns = [ [y_i, -x_i, s_i*c_i, 1] ]
    B = sm.Matrix.zeros(4,4)
    for i in range(4):
        B[0,i] = yy[i]
        B[1,i] = -x[i]
        B[2,i] = s[i]*c[i]
        B[3,i] = 1
    return B

# ---- Least squares thrusts f = argmin ||B f - y||^2  (no bounds) ----
def alloc_thrusts(y, x, yy, s, c):
    B = build_B(x, yy, s, c)
    # Normal equations (pinv-ish): f = (BᵀB)^(-1) Bᵀ y, assuming full rank
    BtB = B.T * B
    Bt_y = B.T * y
    f = BtB.inverse() * Bt_y
    return f

# ---- Map thrusts -> commands via inverse motor model (quadratic) ----
def thrusts_to_cmds(f, k, u0, u_min, u_max):
    u = sm.Vector4.zero()
    for i in range(4):
        # clip thrust to physical range implied by u_min/u_max
        fmin = k[i] * sm.max(0, (u_min[i]-u0[i]))**2
        fmax = k[i] * sm.max(0, (u_max[i]-u0[i]))**2
        f_i = sm.min(sm.max(f[i], fmin), fmax)
        u[i] = sm.min(sm.max(quad_inv(f_i, k[i], u0[i]), u_min[i]), u_max[i])
    return u

def alloc_and_map(y, x, yy, s, c, k, u0, u_min, u_max):
    f = alloc_thrusts(y, x, yy, s, c)
    u = thrusts_to_cmds(f, k, u0, u_min, u_max)
    return u, f

Codegen.function(
    func=alloc_and_map,
    input_types=[sm.Vector4, sm.Vector4, sm.Vector4, sm.Vector4, sm.Vector4,
                 sm.Vector4, sm.Vector4, sm.Vector4, sm.Vector4],
    output_names=["u_cmd","f_alloc"],
    name="alloc_and_map",
).generate_function("generated")
