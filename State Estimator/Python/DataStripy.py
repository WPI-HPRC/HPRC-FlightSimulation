import serial
import csv

# Serial port configuration
port = '/dev/cu.usbmodem1301'  # Change this to your Arduino's serial port
baudrate = 115200  # Change this to match the baudrate of your Arduino

# Open serial port
ser = serial.Serial(port, baudrate)

# Open CSV file for writing
csv_file = open('data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)

try:
    # Write header row to CSV file
    header = ["state", "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ", "rawMagX", "rawMagY", "rawMagZ",
              "pressure", "servoPosition", "altitude", "magX", "magY", "magZ", "w", "i", "j", "k", "posX", "posY",
              "posZ", "velX", "velY", "velZ", "gpsLat", "gpsLong", "gpsAltAGL", "gpsAltMSL", "gpsVelN", "gpsVelE", "gpsVelD", "epochTime", "satellites",
              "gpsLock", "loopCount", "timestamp"]
    csv_writer.writerow(header)

    # Read data from serial port indefinitely
    while True:
        # Read line from serial port
        line = ser.readline().decode().strip()
        
        # Split the line into individual values
        data = line.split(", ")

        dataValues = []
        for datum in data:
            datum = datum.split(":")
            datum = datum[1]
            dataValues.append(datum)



        print(dataValues)

        # Write data to CSV file
        csv_writer.writerow(dataValues)

finally:
    # Close serial port and CSV file
    ser.close()
    csv_file.close()
