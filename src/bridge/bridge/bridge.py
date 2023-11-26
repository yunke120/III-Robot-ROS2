import rclpy
from rclpy.node import Node
import geometry_msgs.msg

import serial
import time
from threading import Lock

# ls /dev/ttyUSB*
# sudo chmod a+rw /dev/ttyUSB0
# ros2 run your_package your_node --ros-args -p your_param:=new_value

class Bridge(Node):
    def __init__(self):
        super().__init__('bridge')
        self.get_logger().info("create bridge node")

        self.declare_parameter('port', value = "/dev/ttyUSB0")
        self.port = self.get_parameter('port').value

        self.declare_parameter('baudrate', value = 115200)
        self.baudrate = self.get_parameter('baudrate').value

        self.declare_parameter('timeout', value=0.2)
        self.timeout = self.get_parameter('timeout').value

        self.timer = self.create_timer(1.0, self.on_timer)

        self.mutex = Lock()


        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )   
        print(f"Connected to {self.ser}")

        self.count = 0
    def on_timer(self):
        self.count += 1
        cmd_list = [self.count, self.count+1,self.count+2,self.count+3,self.count+4]
        self.send_cmd(cmd_list)

    def recv_cmd(self):
        self.mutex.acquire()
        try:
            data = self.ser.readline()
            hex_list = [hex(c) for c in data]
            print(hex_list)
        finally:
            self.mutex.release()

    def send_cmd(self, cmd_list):
        self.mutex.acquire()
        try:
            cmd_bytes = bytes(cmd_list)
            self.ser.write(cmd_bytes)

        finally:
            self.mutex.release()
        pass
    def close_serial(self):
        self.ser.close()

def main():
    rclpy.init()
    node = Bridge()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        node.recv_cmd()

    node.close_serial()
    rclpy.shutdown()

if __name__ == '__main__':
    main()