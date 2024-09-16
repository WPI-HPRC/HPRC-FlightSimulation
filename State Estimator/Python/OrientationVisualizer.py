import pygame
from pygame.locals import *
import numpy as np
import struct
import socket
import serial
import binascii
import math
from time import sleep

inputType = "SERIAL"

if inputType == "UDP":
    ## UDP Server Info
    SERVER_IP = "127.0.0.1"
    SERVER_PORT = 18550
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SERVER_IP, SERVER_PORT))
else:
    ser = serial.Serial('/dev/cu.usbmodem11301', 115200)

## Initialize Viewer
def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    rotation_matrix = np.array([
        [qw**2 + qx**2 - qy**2 - qz**2, 2 * (qx*qy - qw*qz), 2 * (qx*qz + qw*qy)],
        [2 * (qx*qy + qw*qz), qw**2 - qx**2 + qy**2 - qz**2, 2 * (qy*qz - qw*qx)],
        [2 * (qx*qz - qw*qy), 2 * (qw*qx + qy*qz), qw**2 - qx**2 - qy**2 + qz**2]
    ])
    return rotation_matrix

def quaternion_to_euler(q):
    rotation_matrix = quaternion_to_rotation_matrix(q)
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)

    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = math.atan2(-rotation_matrix[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw

pygame.init()
width, height = 1280, 720
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("3D Axis Visualization")
font = pygame.font.Font(None, 36)

bodyAccel = [0,0,0]
quaternion = [0,0,0,0]
rxLinearAccel = [0,0,0]
magVector = [0,0,0]
velVector = [0,0,0]

counter = 0

printAfter = 0

running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    if inputType == "UDP":
        # Receive data from UDP Server
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        quaternion = np.frombuffer(data, dtype=np.float32)
    else:
        data = ser.readline()
        try:
            keys = ["ACC", "QUAT", "LINACCEL_B", "MAGV", "VEL"]
            dataArr = data.decode().split('|')
            if(dataArr[0] in keys):
                key = dataArr[0]

                if(key == "ACC"):
                    bodyAccel = np.array([float(num) for num in dataArr[1].split(',')])
                elif(key == "QUAT"):
                    quaternion = np.array([float(num) for num in dataArr[1].split(',')])
                elif(key == "LINACCEL_B"):
                    rxLinearAccel = np.array([float(num) for num in dataArr[1].split(',')])
                elif(key == "MAGV"):
                    magVector = np.array([float(num) for num in dataArr[1].split(',')])
                elif(key == "VEL"):
                    velVector = np.array([float(num) for num in dataArr[1].split(',')])
            else:
                print("Unknown Packet: ")
                print(dataArr)
                continue
        except ValueError:
            print(data.decode())
            continue

    # Clear the screen
    screen.fill((255, 255, 255))

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)

    # Extract Euler angles from quaternion
    roll, pitch, yaw = quaternion_to_euler(quaternion)

    axis_length = 100

    # Transform axes using rotation matrix
    x_axis_transformed = np.dot(rotation_matrix, np.array([axis_length, 0, 0]))
    y_axis_transformed = np.dot(rotation_matrix, np.array([0, axis_length, 0]))
    z_axis_transformed = np.dot(rotation_matrix, np.array([0, 0, axis_length]))

    # Draw the transformed axes
    pygame.draw.line(screen, (255, 0, 0), (width // 2, height // 2), (width // 2 + z_axis_transformed[0], height // 2 - z_axis_transformed[1]), 2)  # Z-axis (vertical)
    pygame.draw.line(screen, (0, 255, 0), (width // 2, height // 2), (width // 2 + x_axis_transformed[0], height // 2 - x_axis_transformed[1]), 2)  # X-axis (lateral)
    pygame.draw.line(screen, (0, 0, 255), (width // 2, height // 2), (width // 2 + y_axis_transformed[0], height // 2 - y_axis_transformed[1]), 2)  # Y-axis (longitudinal)

    # Display quaternion and Euler angles
    
    quat_text = font.render(f"Quaternion: {quaternion}", True, (0, 0, 0))
    eul_text = font.render(f"Euler Angles: {round(roll*(180/math.pi))}, {round(pitch*(180/math.pi))}, {round(yaw*(180/math.pi))+180}", True, (0, 0, 0))

    acc_text = font.render(f"Velocity Vector: {velVector}", True, (0,0,0))

    screen.blit(quat_text, (10,10))
    screen.blit(eul_text, (10,35))
    screen.blit(acc_text, (10,60))

    pygame.display.flip()

pygame.quit()
