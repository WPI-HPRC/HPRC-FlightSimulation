import numpy as np
import math

class StateEstimator:

    # Constructor
    def __init__(self, dt, x0):
        self.dt = dt
        self.x = x0

    # State Vector
    ## x = [w, i, j, k]
    x = np.array([0,0,0,0])

    # Error Covariance Matrix
    P = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    P_min = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Sensor Variance
    gyroNoise = math.pow(0.00262,2) # Gyro Noise
    R = np.array([
        [gyroNoise, 0, 0],
        [0, gyroNoise, 0],
        [0, 0, gyroNoise]
    ])

    # Process Noise Covariance
    Q = np.array([
        [0.001, 0, 0, 0],
        [0, 0.001, 0, 0],
        [0, 0, 0.001, 0],
        [0, 0, 0, 0.001]
    ]) 

    def measurementFunction(self, x, u, dt):
        p = u[0]; q = u[1]; r = u[2]

        f = np.array([
            x[0] - (dt/2)*p*x[1] - (dt/2)*q*x[2] - (dt/2)*r*x[3],
            x[1] + (dt/2)*p*x[0] - (dt/2)*q*x[3] + (dt/2)*r*x[2],
            x[2] + (dt/2)*p*x[3] + (dt/2)*q*x[0] - (dt/2)*r*x[1],
            x[3] - (dt/2)*p*x[2] + (dt/2)*q*x[1] + (dt/2)*r*x[0]
        ])

        # dx = np.array([
        #     [0, -0.5*p, -0.5*q, -0.5*r],
        #     [0.5*p, 0, 0.5*r, -0.5*q],
        #     [0.5*q, -0.5*r, 0, 0.5*p],
        #     [0.5*r, 0.5*q, -0.5*p, 0]
        # ])

        # # f = self.multiply_quaternions(x, np.array([0, -0.5*p, -0.5*q, -0.5*r]))

        # f = np.dot(dx, x) / np.linalg.norm(x)

        return f

    def measurementJacobian(self, x, u):
        p = u[0]; q = u[1]; r = u[2]

        A = np.array([
        [0, -0.5*p, -0.5*q, -0.5*r],
        [0.5*p, 0, 0.5*r, 0.5*q],
        [0.5*q, -0.5*r, 0, 0.5*p],
        [0.5*r, 0.5*q, -0.5*p, 0]
        ])

        return A
    
    def onLoop(self, sensorData, dt):

        u = sensorData

        # x_min = self.x + self.measurementFunction(self.x, u) * dt
        x_min = self.measurementFunction(self.x, u, dt)

        A = self.measurementJacobian(x_min, u)

        self.P_min = A*self.P*A.T + self.Q*dt

        self.x = x_min
        self.P = self.P_min


        return self.x
    
    def quaternion_to_euler(self, q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).

        Parameters:
        - q: tuple or list representing the quaternion (w, x, y, z)

        Returns:
        - euler_angles: tuple (roll, pitch, yaw) in radians
        """
        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]


        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x**2 + y**2)
        roll_x = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y**2 + z**2)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z