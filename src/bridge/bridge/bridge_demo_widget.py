import tkinter as tk
import rclpy
from std_msgs.msg import ByteMultiArray
from rclpy.node import Node
import struct
from bridge.protocol import eDEVICE, eRRobot
from enum import Enum, IntEnum

class UInt16Parts:
    def __init__(self, low_byte, high_byte):
        self.low_byte = low_byte
        self.high_byte = high_byte

class UInt16Union:
    def __init__(self, value):
        if not isinstance(value, int):
            raise ValueError("Value must be an integer.")
        self.value = value

    def get_parts(self):
        bytes_representation = struct.pack('>H', self.value)
        low_byte, high_byte = struct.unpack('BB', bytes_representation)
        
        return UInt16Parts(low_byte, high_byte)

class BridgeDemoWidget(Node):

    def __init__(self):
        super().__init__('bridge_demo_widget')
        self.publisher = self.create_publisher(ByteMultiArray, "/stm32/recv", 10)
        self.msg = ByteMultiArray()
        self.subscriber = self.create_subscription(ByteMultiArray, "/stm32/send", self.sub_cb, 10)
    def publish_message(self, cmd):
        if len(cmd) != 12:
            print("\033[91mError: The command length is not equal to 12.\033[0m")
        data_bytes = [i.to_bytes(1, 'big') for i in cmd]
        self.msg.data = data_bytes
        self.get_logger().info(f'Publishing: {cmd}')
        self.publisher.publish(self.msg)
    def sub_cb(self, msg):
        print(msg.data)

class App:

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.create_widgets()

    def create_widgets(self):
        self.root.geometry("300x200+200+200")
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

        self.label = tk.Label(self.root, text="Velocity").grid(row=3,column=0,sticky="nsew", padx = 1, pady=3)
        self.velocity_entry = tk.Entry(self.root)
        self.velocity_entry.grid(row=3,column=1, sticky="nsew", padx = 10, pady=10)
        self.set_v_button = tk.Button(self.root, text="set", command=self.set_v_button_func)
        self.set_v_button.grid(row=3,column=2, sticky="nsew", padx = 10, pady=10)

        self.root.columnconfigure([0,1,2], weight=1)
        self.root.rowconfigure([0,1,2,3], weight=1)
    
    def forward_button_func(self):
        cmd_list = [0xAA, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = self.check_data(cmd_list)
        cmd_list[-2] = check
        print(cmd_list)
        self.node.publish_message(cmd_list)
    def stop_button_func(self):
        cmd_list = [0xAA, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDD]
        check = self.check_data(cmd_list)
        cmd_list[-2] = check
        print(cmd_list)
        self.node.publish_message(cmd_list)
    def backward_button_func(self):
        pass
    def left_button_func(self):
        pass
    def right_button_func(self):
        pass
    def set_v_button_func(self):
        text = self.velocity_entry.get()
        try:
            fval = float(text)
            print(f'fval:{fval}')
        except ValueError:
            print("Invalid input. Please enter a valid number.")
        v = int(fval * 1000)
        data = UInt16Union(v).get_parts()
        print(f"Original Number: {v}")
        print(f"High 8 bits: 0x{data.high_byte:02X}")  # 以16进制打印
        print(f"Low 8 bits: 0x{data.low_byte:02X}")   # 以16进制打印
        cmd_list = [0xAA, eDEVICE.Robot.value, eRRobot.R_RobotVelocity.value, 0x02, data.high_byte, data.low_byte, 0,0,0,0, 0,0xDD]
        check = self.check_data(cmd_list)
        cmd_list[-2] = check
        print(cmd_list)
        self.node.publish_message(cmd_list)

    def check_data(self, buf):
        sum_value = 0
        for item in buf[:-2]:  # 排除最后两个元素
            if isinstance(item, Enum):  # 如果是枚举类型，转换为整数
                sum_value ^= int(item.value)
            else:
                sum_value ^= item
        return sum_value

def main():
    rclpy.init()
    node = BridgeDemoWidget()

    root = tk.Tk()
    app = App(root, node)

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0)
        root.after(10, spin_ros)  # Schedule the next spin

    root.after(10, spin_ros)  # Schedule the first spin
    root.mainloop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
