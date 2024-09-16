import sys
import time
import socket
import numpy as np
import math

import xpc
from EKF import StateEstimator

import pygame
from pygame.locals import *

## UDP Server Info
SERVER_IP = "127.0.0.1"
SERVER_PORT = 18550
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def monitor():
    with xpc.XPlaneConnect() as client:

        # Initialize EKF
        ## dt = 0.01
        ## Initial State = [1,0,0,0]
        ## State = [w, i, j, k]
        ekf = StateEstimator(0.01, np.array([1,0,0,0]))

        startTime = time.time()
        loopTime = 0.0
        while True:
            # Obtain gyro rates from X-Plane and convert to rad/s
            p_value = float(client.getDREF("sim/flightmodel/position/P")[0]) * (math.pi / 180.0)
            q_value = float(client.getDREF("sim/flightmodel/position/Q")[0]) * (math.pi / 180.0)
            r_value = float(client.getDREF("sim/flightmodel/position/R")[0]) * (math.pi / 180.0)

            # accelX = client.getDREF("sim/flightmodel/forces/g_axil")[0] # [G]
            # accelY = client.getDREF("sim/flightmodel/forces/g_side")[0] # [G]
            # accelZ = client.getDREF("sim/flightmodel/forces/g_nrml")[0] # [G]
            
            # pressure = client.getDREF("sim/weather/barometer_current_inhg")[0] * 3386 # [Pa]

            # heading = client.getDREF("sim/flightmodel/position/mag_psi")[0] # [deg]

            sensorData = np.array([p_value, q_value, r_value])
            currState = ekf.onLoop(sensorData, loopTime)
            print(currState)
            # roll,pitch,yaw = quaternion_to_euler_angles(currState)
            # print(roll, pitch, yaw)

            endTime = time.time()
            loopTime = endTime - startTime
            startTime = endTime

            sock.sendto(currState.tobytes(), (SERVER_IP, SERVER_PORT))

            time.sleep(0.01)

def pressureToAlt(pressure):
    pb = 101325 # [Pa] Pressure at sea level
    Tb = 288.15 # [K] Temperature at sea level
    Lb = -0.0065 # [K/m] standard temperature lapse rate
    hb = 0 # [m] height at sea level
    R = 8.3142 # [N*m/mol*K] Universal Gas Constant
    g0 = 9.80665 # [m/s/s] Earth standard gravity
    M = 0.0289644 # [kg/mol] Molar mass of Earth's air

    return hb + (Tb / Lb) * (pow((pressure/pb), (-R*Lb / (g0*M))) - 1)

def quaternion_to_euler_angles(quaternion):
  # Extract the x, y, z, and w components of the quaternion.
  w = quaternion[0]
  i = quaternion[1]
  j = quaternion[2]
  k = quaternion[3]

  # Calculate the Euler angles.
#   roll = np.arctan2(2*(w*i+j*k), 1-2*(pow(i,2)+pow(j,2))) * (180/math.pi)
#   pitch = (-math.pi/2)+2*np.arctan2(math.sqrt(1+2*(w*j-i*k)), math.sqrt(1-2*(w*j-i*k))) * (180/math.pi)
#   yaw = np.arctan2(2*(w*k+i*j), 1-2*(pow(j,2)+pow(k,2))) * (180/math.pi)
  yaw = np.arctan2(2 * (i * j + k * w), 1 - 2 * (j ** 2 + k ** 2)) * (180/math.pi)
  pitch = np.arctan2(2 * (i * k - j * w), 1 - 2 * (i ** 2 + j ** 2)) * (180/math.pi)
  roll = np.arctan2(2 * (j * k + i * w), 1 - 2 * (i ** 2 + k ** 2)) * (180/math.pi)

  # Return the Euler angles.
  return roll, pitch, yaw

if __name__ == "__main__":
    monitor()