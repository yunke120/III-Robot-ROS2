
'''
RUN:
	$ sudo chmod a+rw /dev/ttyUSB0
	$ python3 test_serial.py

Attention:
  	超时时间timeout要和发送数据端发送时间间隔搭配
'''

import serial

def on_new_data(data):
    num_list = [int(char) for char in data]
    print(num_list)

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

try:
    while True:
        line = ser.readline().decode('utf-8')
        on_new_data(line)

except KeyboardInterrupt:
    pass

finally:
    ser.close()