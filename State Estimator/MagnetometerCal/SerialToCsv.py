import serial
import csv

ser = serial.Serial('COM5', 115200)

filename = "polarisData.csv"

# Open the CSV file for writing
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)

    # Write the header row
    writer.writerow(["magX", "magY", "magZ"])

    while True:
        data = ser.readline()

        # Convert the data to a list of integers
        numbers = [float(num) for num in data.decode().split(',')]
        magX, magY, magZ = numbers

        # print(magX, magY, magZ)

        # Write the data to the CSV file
        writer.writerow([magX, magY, magZ])