
import rclpy
from bot_programing.bot_action import Bot_Cmd

def main(args=None):
    rclpy.init(args=args)
    bot_cmd = Bot_Cmd()
    bot_cmd.move_map(0.3, 0.0, 0.0)
    exit(0)

if __name__ == '__main__':
    main()