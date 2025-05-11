import numpy as np

class Quadrotor:
    def __init__(self, mass = 1.0, r_P = np.array([10; 0; 0]), J = np.diag([0.005, 0.005, 0.01]), g = 9.81):
        self.J = J
        self.m = m
        self.r_P = r_P
        self.g = g

        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1, 0, 0, 0])  
        self.omega = np.zeros(3)

    def dyn(self, u, dt):
        thrust, tau_x, tau_y, tau_z = inputs

        # Extract state variables
        x, y, z, vx, vy, vz, roll, pitch, yaw, p, q, r = self.state

        # Rotation matrix from body to world frame
        R = np.array([
            [np.cos(yaw) * np.cos(pitch), np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
            [np.sin(yaw) * np.cos(pitch), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll), np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
            [-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll)]
        ])

        # Translational dynamics
        accel = np.array([0, 0, -self.gravity]) + (R @ np.array([0, 0, thrust / self.mass]))
        velocity = np.array([vx, vy, vz]) + accel * dt
        position = np.array([x, y, z]) + velocity * dt

        # Rotational dynamics
        angular_velocity = np.array([p, q, r])
        torques = np.array([tau_x, tau_y, tau_z])
        angular_accel = np.linalg.inv(self.inertia) @ (torques - np.cross(angular_velocity, self.inertia @ angular_velocity))
        angular_velocity += angular_accel * dt

        # Update angles
        roll += p * dt
        pitch += q * dt
        yaw += r * dt

        # Update state
        self.state = np.hstack((position, velocity, roll, pitch, yaw, angular_velocity))

    def get_state(self):
        """
        Get the current state of the quadrotor.
        """
        return self.state