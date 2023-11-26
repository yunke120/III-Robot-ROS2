from enum import Enum, IntEnum

class eDEVICE(Enum):
    DEVNone = 0x00
    Robot = 1
    PTZ = 2
    RoboticArm = 3
    Battery = 4
    IMU = 5
    Sensor = 6

class eRRobot(Enum):
    R_RobotVelocity = 0x20
    HandShake = 0xFF