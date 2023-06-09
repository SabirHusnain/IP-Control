import serial
import time
import csv

f = open("data.csv", 'w', newline='')
f.truncate()
writer = csv.writer(f, delimiter=',')

esp = serial.Serial('COM3', 115200, timeout=0.05)
i = 0
esp.setDTR(False)
time.sleep(1)
esp.flushInput()
time.sleep(1)
esp.setDTR(True)
while (i <= 6000):
    try:
        serial_data = esp.readline()
        decoded_data = serial_data.decode("utf-8").strip('\r\n')

        data = [float(x) for x in decoded_data.split(',')]

        writer.writerow(data)

    except:
        print("Error!")
    i += 1
f.close()

print('Done!')
