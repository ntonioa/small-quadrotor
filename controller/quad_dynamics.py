import symforce
symforce.set_epsilon_to_symbol()
import symforce.symbolic as sf
from symforce import codegen

@sf.symforce_function
def quad_dynamics(
    q: sf.Quaternion,            # Quaternion [w, x, y, z]
    omega: sf.V3,                # Angular velocity in body frame
    pos: sf.V3,                  # Position in world
    vel: sf.V3,                  # Velocity in world
    u_thrust: sf.Scalar,        # Total thrust along body z
    u_tau: sf.V3,                # Torque in body frame
    J: sf.Matrix33,             # Inertia matrix in body frame
    mass: sf.Scalar             # Mass of quadrotor
) -> sf.Tuple:
    
    # Gravity
    g = 9.81
    gravity = sf.V3(0, 0, -mass * g)

    # Quaternion derivative: q̇ = 0.5 * (ω_quat * q)
    omega_quat = sf.Quaternion(0.0, omega[0], omega[1], omega[2])
    q_dot = 0.5 * omega_quat * q

    # Torque equation: ω̇ = J⁻¹ (τ - ω × (Jω))
    omega_cross = omega.cross(J * omega)
    omega_dot = J.inverse() * (u_tau - omega_cross)

    # Acceleration: a = (R * thrust + gravity) / mass
    thrust_world = q.rotate(sf.V3(0, 0, u_thrust))
    accel = (thrust_world + gravity) / mass

    # Output derivatives: [q̇, ω̇, ṗ, v̇]
    return (q_dot, omega_dot, vel, accel)