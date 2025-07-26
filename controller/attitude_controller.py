# attitude_controller.py

import symforce
symforce.set_epsilon_to_symbol()
import symforce.symbolic as sf

@sf.symforce_function
def attitude_pd_controller(
    q: sf.Quaternion,
    q_des: sf.Quaternion,
    omega: sf.V3,
    omega_des: sf.V3,
    Kp: sf.Scalar,
    Kd: sf.Scalar
) -> sf.V3:

    # Rotation error in tangent space (shortest path)
    q_err = q_des.inverse() * q
    if q_err.w() < 0:
        q_err = -q_err  # Ensure shortest path

    e_rot = 2 * q_err.vec()  # rotation error (small angle approx)

    # Angular velocity error
    e_omega = omega - omega_des

    # Torque output
    torque = -Kp * e_rot - Kd * e_omega
    return torque
