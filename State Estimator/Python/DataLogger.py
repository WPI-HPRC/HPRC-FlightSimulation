import serial
import io
import numpy as np
import math
import time
import csv

ser = serial.Serial('/dev/cu.usbmodem11301', 115200)

# Read Data and Parse
filename = "polarisData.csv"

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)

    # Write the header row
    writer.writerow(["AccelX", "AccelY", "AccelZ", "GyroX", "GyroY", "GyroZ", "MagX", "MagY", "MagZ", "w", "i", "j", "k", "timestamp"])

    while True:
        data = ser.readline()

        try:
            numbers = [float(num) for num in data.decode().split(',')]
            acx,acy,acz,gyx,gyy,gyz,magx,magy,magz,w,i,j,k,timestamp = numbers

            writer.writerow([acx,acy,acz,gyx,gyy,gyz,magx,magy,magz,w,i,j,k,timestamp])
        except ValueError:
            print("ERROR: ")
            print(data.decode())
            continue

    