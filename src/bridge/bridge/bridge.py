# -*- coding: utf-8 -*-
"""
这个模块用来作为上位机控制端与STM32通信的桥梁, 它可以接收STM32的消息并转发给上位机, 
同时接收上位机的消息转发给STM32, 并且处理自身的消息进行转发

代码实现：
与STM32通信使用的是microros,因此需要建立一个订阅者和发布者
与上位机通信使用的是数传通信, 同样建立在串口基础上, 故使用pyserial进行实现

使用：
    $ sudo chmod a+rw /dev/ttyUSB0
    $ ros2 run bridge bridge
    附带参数：
    $ ros2 run your_package your_node --ros-args -p your_param:=new_value

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from bridge.protocol import eDEVICE
import serial
from threading import Lock
from enum import Enum

def check_data(buf):
    """ 数据校验 """
    sum_value = 0
    for item in buf[:-2]:  # 排除最后两个元素
        if isinstance(item, Enum):  # 如果是枚举类型，转换为整数
            sum_value ^= int(item.value)
        elif isinstance(item, bytes):  # 如果是 bytes 类型，转换为整数
            sum_value ^= int.from_bytes(item, byteorder='big')
        else:
            sum_value ^= item
    return sum_value


class UInt16Extractor:
    """ 对大于255的数据需要通过16位数据进行保存
        单片机接收的是uint8_t类型, 因此需要对数据进行解包, 分为高位和低8位 
        同时对接收的数据进行封包
    """
    def __init__(self):
        pass
    def unpack(self, value):
        """ 解包
        arg: value
            value: 大于255的数据
        """
        if not isinstance(value, int):
            raise ValueError("Value must be an integer.")
        low_byte = value&0xFF
        high_byte = (value>>8)&0xFF
        return low_byte, high_byte

    def pack(self, low, high):
        """ 封包 
        args:
            low: 数据低8位
            high: 数据高8位
        """
        # 使用 int.from_bytes 将 bytes 转换为整数
        low_value = int.from_bytes(low, byteorder='big')
        high_value = int.from_bytes(high, byteorder='big')
        value = (high_value << 8) | low_value
        return value
    
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

        self.publisher = self.create_publisher(ByteMultiArray, "/stm32/recv", 10) 
        self.subscriber = self.create_subscription(ByteMultiArray, "/stm32/send", self.sub_cb, 10)
        self.extractor = UInt16Extractor()

        self.msg = ByteMultiArray()

        self.mutex = Lock()

        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )   
        print(f"Connected to {self.ser}")

    def sub_cb(self, msg):
        print(msg.data)
        check = check_data(msg.data)
        if check != int.from_bytes(msg.data[-2], byteorder='big'):
            if int.from_bytes(msg.data[1], byteorder='big') == eDEVICE.Robot.value: # Robot device
                print(msg.data)
            else:
                print("Other device")
        else:
            print("\033[91mError: Data verification failed.\033[0m")

    def recv_from_pc(self):
        """ 接收上位机消息 """
        self.mutex.acquire()
        try:
            data = self.ser.readline()
            # hex_list = [hex(c) for c in data]
            # print(hex_list)
            if len(data) == 12:
                print(data, type(data))
                if data[1] == eDEVICE.Robot.value:  # Robot
                    data_bytes = [c.to_bytes(1, 'big') for c in data]
                    print(data_bytes)
                    self.msg.data = data_bytes
                    self.publisher.publish(self.msg)
                else:
                    print('none')
        finally:
            self.mutex.release()

    def send_to_pc(self, cmd_list):
        """ 向上位机发送消息
        参数
        ---
        arg: cmd_list
            cmd_lists是一个列表, 并且每一个参数是整数, 例如 cmd_list[1,2,3,4,5,6]
        """
        self.mutex.acquire()
        try:
            cmd_bytes = bytes(cmd_list)
            self.ser.write(cmd_bytes)

        finally:
            self.mutex.release()

    def close_serial(self):
        """ 关闭串口 """
        self.ser.close()

def main():
    rclpy.init()
    node = Bridge()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        node.recv_from_pc()

    node.close_serial()
    rclpy.shutdown()

if __name__ == '__main__':
    main()