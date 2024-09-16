import numpy as np
import serial
import os
import math

def init():
    print("[INIT] Starting System...")

    x0 = np.array([1,0,0,0,0,0,0,0,0,0])

    estimator = StateEstimator()


class StateEstimator:
    def __init__(self):
        self.bodyAccel = [0,0,0]
        self.rxLinearAccel = [0,0,0]
        self.magVector = np.array([0,0,0])
        self.accelVector = np.array([0,0,0])
        self.gyroVector = np.array([0,0,0])
        self.ekfInitialized = False

        self.initSerial()

    def initSerial(self):

        # Scan Serial Ports and Prompt User to Select
        ports = []
        for i in range(256):
            try:
                s = serial.Serial(f"COM{i}", 115200)
                ports.append(i)
                s.close()
            except serial.SerialException:
                pass
        
        print("Available Ports: ")
        if(len(ports) == 0):
            print("No Ports Available...")
            exit(0)

        for i in range(len(ports)):
            print(f"[{i}] COM{ports[i]}")

            port = int(input("Select Port: "))
            self.ser = serial.Serial(f"COM{ports[port]}", 115200)

            self.receieveData()

    def receieveData(self):
        while True:
            data = self.ser.readline().decode('utf-8').splitlines()[0]
            
            # Split Sensor Readings
            dataArr = data.split(',')
            self.accelVector = dataArr[0:3]
            self.gyroVector  = dataArr[3:6]
            self.magVector   = dataArr[6:10]

            # Convert to float
            self.accelVector = np.array([float(num) for num in self.accelVector])
            self.gyroVector  = np.array([float(num) for num in self.gyroVector])
            self.magVector   = np.array([float(num) for num in self.magVector])

            if(not self.ekfInitialized):
                self.initEKF()

    def initEKF(self):
        # Calculate Initial Quaternion
        print("[EKF] Initializing EKF...")

        accel_norm = self.accelVector / np.linalg.norm(self.accelVector)
        mag_norm = self.magVector / np.linalg.norm(self.magVector)

        # Step 2: Assume up vector (direction of gravity)
        up_vector = np.array([0, 0, 1])  # Assuming gravity acts along the z-axis

        # Step 3: Calculate east vector (cross product of up and accel)
        east_vector = np.cross(accel_norm, mag_norm)
        east_vector /= np.linalg.norm(east_vector)

        # Step 4: Calculate north vector (cross product of east and accel)
        north_vector = np.cross(east_vector, accel_norm)
        north_vector /= np.linalg.norm(north_vector)

        # Step 5: Construct quaternion
        w = np.sqrt(1.0 + east_vector[0] + north_vector[1] + up_vector[2]) / 2.0
        w4 = (4.0 * w)
        x = (north_vector[2] - east_vector[1]) / w4
        y = (east_vector[0] - accel_norm[2]) / w4
        z = (accel_norm[1] - north_vector[0]) / w4

        initial_quaternion = np.array([w, x, y, z])

        # Step 6: Normalize the quaternion
        initial_quaternion /= np.linalg.norm(initial_quaternion)

        heading = np.arctan2(2 * (x * y + w * z), w**2 - x**2 - y**2 + z**2)
        heading_degrees = np.degrees(heading)
        if heading_degrees < 0:
            heading_degrees += 360  # Convert negative angle to positive
        
        print(f"Initial Heading: {heading_degrees} degrees")

        print(initial_quaternion)

        self.ekfInitialized = True

if __name__ == '__main__':
    init()