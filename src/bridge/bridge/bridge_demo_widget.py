# -*- coding: utf-8 -*-
"""
这个模块用来测试通过ROS2控制STM32端的界面, 具有控制小车上下左右, 加减速, 以及显示实时速度和电池电压的功能

使用：
    $ ros2 run bridge bridge_demo_widget

"""

import tkinter as tk
import rclpy
from std_msgs.msg import ByteMultiArray
from rclpy.node import Node
from bridge.protocol import eDEVICE, eRRobot
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

class BridgeDemoWidget(Node):

    def __init__(self):
        super().__init__('bridge_demo_widget')
        self.publisher = self.create_publisher(ByteMultiArray, "/stm32/recv", 10)
        self.msg = ByteMultiArray()
        self.subscriber = self.create_subscription(ByteMultiArray, "/stm32/send", self.sub_cb, 10)
        self.extractor = UInt16Extractor()
        self.velocity = 0.0
        self.voltage = 0.0
    def publish_message(self, cmd):
        
        if len(cmd) != 12:
            print("\033[91mError: The command length is not equal to 12.\033[0m")
        data_bytes = [i.to_bytes(1, 'big') for i in cmd]
        self.msg.data = data_bytes
        self.get_logger().info(f'Publishing: {cmd}')
        self.publisher.publish(self.msg)
    def sub_cb(self, msg):
        print(msg.data)
        check = check_data(msg.data)
        if check == int.from_bytes(msg.data[-2], byteorder='big'):
            velocity = self.extractor.pack(msg.data[1], msg.data[2])
            voltage = self.extractor.pack(msg.data[3], msg.data[4])
            self.velocity = round(velocity/1000, 3)
            self.voltage = round(voltage/1000, 3)
            # print(f'{self.velocity}, {self.voltage}')
        else:
            print("\033[91mError: Data verification failed.\033[0m")

class App:

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.create_widgets()
        self.extractor = UInt16Extractor()

    def create_widgets(self):
        self.root.geometry("350x200+200+200")
        self.root.title("bridge demo widget")

        self.forward_button = tk.Button(self.root, text="forward", command=self.forward_button_func)
        self.forward_button.grid(row=0, column=1, sticky="nsew")
        self.stop_button = tk.Button(self.root, text="stop", bg='#E58686', command=self.stop_button_func)
        self.stop_button.grid(row=1,column=1, sticky="nsew")

        self.backward_button = tk.Button(self.root, text="backward", command=self.backward_button_func)
        self.backward_button.grid(row=2,column=1, sticky="nsew")
        self.left_button = tk.Button(self.root, text="left", command=self.left_button_func)
        self.left_button.grid(row=1,column=0, sticky="nsew")
        self.right_button = tk.Button(self.root, text="right", command=self.right_button_func)
        self.right_button.grid(row=1,column=2, sticky="nsew")

        self.label = tk.Label(self.root, text="SetVelocity").grid(row=3,column=0,sticky="nsew", padx = 1, pady=3)
        self.velocity_entry = tk.Entry(self.root)
        self.velocity_entry.grid(row=3,column=1, sticky="nsew", padx = 10, pady=10)
        self.set_v_button = tk.Button(self.root, text="set", command=self.set_v_button_func)
        self.set_v_button.grid(row=3,column=2, sticky="nsew", padx = 10, pady=10)

        self.label_vel = tk.Label(self.root, text="vel:m/s")
        self.label_vel.grid(row=4,column=0,sticky="nsew", padx = 5, pady=3)
        self.label_vol = tk.Label(self.root, text="vol:V")
        self.label_vol.grid(row=4,column=2,sticky="nsew", padx = 5, pady=3)

        self.root.columnconfigure([0,1,2], weight=1)
        self.root.rowconfigure([0,1,2,3,4], weight=1)
    
    def forward_button_func(self):
        cmd_list = [0xAA, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = check_data(cmd_list)
        cmd_list[-2] = check
        self.node.publish_message(cmd_list)
    def stop_button_func(self):
        cmd_list = [0xAA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = check_data(cmd_list)
        cmd_list[-2] = check
        self.node.publish_message(cmd_list)
    def backward_button_func(self):
        cmd_list = [0xAA, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = check_data(cmd_list)
        cmd_list[-2] = check
        self.node.publish_message(cmd_list)
    def left_button_func(self):
        cmd_list = [0xAA, 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = check_data(cmd_list)
        cmd_list[-2] = check
        self.node.publish_message(cmd_list)
    def right_button_func(self):
        cmd_list = [0xAA, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = check_data(cmd_list)
        cmd_list[-2] = check
        self.node.publish_message(cmd_list)
    def set_v_button_func(self):
        text = self.velocity_entry.get()
        try:
            fval = float(text)
        except ValueError:
            print("Invalid input. Please enter a valid number.")
        v = int(fval * 1000)
        high_byte, low_byte = self.extractor.unpack(v)
        # print(f"Original Number: {v}")
        # print(f"High 8 bits: 0x{high_byte:02X}")  # 以16进制打印
        # print(f"Low 8 bits: 0x{low_byte:02X}")   # 以16进制打印
        cmd_list = [0xAA, eDEVICE.Robot.value, eRRobot.R_RobotVelocity.value, 0x02, high_byte, low_byte, 0,0,0,0, 0,0xDD]
        check = check_data(cmd_list)
        cmd_list[-2] = check
        self.node.publish_message(cmd_list)

    def update_info(self):
        vel = self.node.velocity
        vol = self.node.voltage
        self.label_vel.config(text=f'vel: {vel} m/s')
        self.label_vol.config(text=f'vol: {vol} V')

def main():
    rclpy.init()
    node = BridgeDemoWidget()
    root = tk.Tk()
    app = App(root, node)

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0)
        root.after(10, spin_ros)  # Schedule the next spin
        app.update_info()

    root.after(10, spin_ros)  # Schedule the first spin
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
