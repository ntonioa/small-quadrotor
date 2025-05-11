import numpy as np

def hat(q):
    q0, q1, q2, q3 = q
    return np.array([
        [q0, -q1, -q2, -q3],
        [q1, q0, q3, -q2],
        [q2, -q3, q0, q1],
        [q3, q2, -q1, q0]
    ])

def vee(H):
    return np.array([H[0, 0], H[1, 0], H[2, 0], H[3, 0]])

def rot(v, q):
    q0, q1, q2, q3 = q
    q_vec = np.array([q1, q2, q3])
    v = np.array(v)

    t = 2 * np.cross(q_vec, v)
    rotated_v = v + q0 * t + np.cross(q_vec, t)
    return rotated_v

def q_inv(q):
    q0, q1, q2, q3 = q
    return np.array([q0, -q1, -q2, -q3])