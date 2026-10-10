import struct

params_size=29

# main board
class MessageID:
    ID_GET_VERSION = 0
    ID_SET_ROBOT_PARAMETER = 1
    ID_GET_ROBOT_PARAMETER = 2
    ID_INIT_ODOM = 3
    ID_SET_VEL = 4
    ID_GET_ODOM = 5
    ID_GET_PID_DEBUG = 6
    ID_GET_IMU = 7
    ID_GET_ENCODER_COUNT = 8
    ID_SET_MOTOR_PWM = 9

class RobotMessage:
    def pack(self):
        return b''

    def unpack(self):
        return True

class RobotFirmwareInfo(RobotMessage):
    def __init__(self):
        self.version = ''
        self.build_time = ''

    def unpack(self, data):
        try:
            upk = struct.unpack('16s16s', bytes(data))
        except:
            return False
        [self.version, self.build_time] = upk
        return True

class RobotImuType:
    IMU_TYPE_GY65 = 49
    IMU_TYPE_GY85 = 69
    IMU_TYPE_GY87 = 71

class RobotModelType:
    MODEL_TYPE_2WD_DIFF = 1
    MODEL_TYPE_4WD_DIFF = 2
    MODEL_TYPE_3WD_OMNI = 101
    MODEL_TYPE_4WD_OMNI = 102
    MODEL_TYPE_4WD_MECANUM = 201

class RobotParameters():
    def __init__(self, wheel_diameter=0, \
                wheel_track=0, \
                encoder_resolution=0, \
                do_pid_interval=0, \
                kp=0, \
                ki=0, \
                kd=0, \
                ko=0, \
                cmd_last_time=0, \
                max_v_liner_x=0, \
                max_v_liner_y=0, \
                max_v_angular_z=0, \
                imu_type=0, \
                motor_ratio=0, \
                model_type=0, \
                motor_nonexchange_flag=255, \
                encoder_nonexchange_flag=255, \
                ):
        self.wheel_diameter = wheel_diameter
        self.wheel_track = wheel_track
        self.encoder_resolution = encoder_resolution
        self.do_pid_interval = do_pid_interval
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ko = ko
        self.cmd_last_time = cmd_last_time
        self.max_v_liner_x = max_v_liner_x
        self.max_v_liner_y = max_v_liner_y
        self.max_v_angular_z = max_v_angular_z
        self.imu_type = imu_type
        self.motor_ratio = motor_ratio
        self.model_type = model_type
        self.motor_nonexchange_flag = motor_nonexchange_flag
        self.encoder_nonexchange_flag = encoder_nonexchange_flag
        reserve=b'\xff'
        self.reserve=b''
        for i in range(64-params_size):
            self.reserve+=reserve
robotParam = RobotParameters()

class GetRobotParameters(RobotMessage):
    def __init__(self):
        self.param = robotParam

    def unpack(self, data):
        #print(bytes(data), len(bytes(data)))
        upk = struct.unpack('<3H1B8H1B1H3B%ds'%(64-params_size), bytes(data))

        [self.param.wheel_diameter,
         self.param.wheel_track,
         self.param.encoder_resolution,
         self.param.do_pid_interval,
         self.param.kp,
         self.param.ki,
         self.param.kd,
         self.param.ko,
         self.param.cmd_last_time,
         self.param.max_v_liner_x,
         self.param.max_v_liner_y,
         self.param.max_v_angular_z,
         self.param.imu_type,
         self.param.motor_ratio,
         self.param.model_type,
         self.param.motor_nonexchange_flag,
         self.param.encoder_nonexchange_flag,
         self.param.reserve] = upk
        return True

class SetRobotParameters(RobotMessage):
    def __init__(self):
        self.param = robotParam

    def pack(self):
        data = [self.param.wheel_diameter,
                self.param.wheel_track,
                self.param.encoder_resolution,
                self.param.do_pid_interval,
                self.param.kp,
                self.param.ki,
                self.param.kd,
                self.param.ko,
                self.param.cmd_last_time,
                self.param.max_v_liner_x,
                self.param.max_v_liner_y,
                self.param.max_v_angular_z,
                self.param.imu_type,
                self.param.motor_ratio,
                self.param.model_type,
                self.param.motor_nonexchange_flag,
                self.param.encoder_nonexchange_flag,
                self.param.reserve]

        print(data)
        pk = struct.pack('<3H1B8H1B1H3B%ds'%(64-(3*2+1+8*2+1+2+3)), *data)
        return pk

    def unpack(self, data):
        return True

class RobotVel(RobotMessage):
    def __init__(self):
        self.v_liner_x = 0
        self.v_liner_y = 0
        self.v_angular_z = 0

    def pack(self):
        data = [self.v_liner_x,
                self.v_liner_y,
                self.v_angular_z]
        pk = struct.pack('3h', *data)
        return pk

    def unpack(self, data):
        return True

#todo the rest of the message classes
class RobotOdom(RobotMessage):
    def __init__(self):
        self.v_liner_x = 0
        self.v_liner_y = 0
        self.v_angular_z = 0
        self.x = 0
        self.y = 0
        self.yaw = 0

    def unpack(self, data):
        try:
            upk = struct.unpack('<3H2l1H', bytes(data))
        except:
            return False
        [self.v_liner_x,
                self.v_liner_y,
                self.v_angular_z,
                self.x,
                self.y,
                self.yaw] = upk
        return True

class RobotPIDData(RobotMessage):
    def __init__(self):
        self.pid_data = [0]*8

    def unpack(self, data):
        try:
            upk = struct.unpack('<8l', bytes(data))
        except:
            return False
        
        self.pid_data = upk
        return True

class RobotIMU(RobotMessage):
    def __init__(self):
        self.imu = [0]*9

    def unpack(self, data):
        try:
            upk = struct.unpack('<9f', bytes(data))
        except:
            return False
        
        self.imu = upk
        return True

class RobotEncoderCount(RobotMessage):
    def __init__(self):
        self.encoder = [0]*4

    def unpack(self, data):
        try:
            upk = struct.unpack('<4f', bytes(data))
        except:
            return False
        
        self.encoder = upk
        return True

class RobotMotorPWM(RobotMessage):
    def __init__(self):
        self.pwm = [0]*4

    def pack(self):
        pk = struct.pack('4h', *self.pwm)
        return pk

    def unpack(self, data):
        return True

BoardDataDict = {MessageID.ID_GET_VERSION:RobotFirmwareInfo(),
            MessageID.ID_GET_ROBOT_PARAMETER:GetRobotParameters(),
            MessageID.ID_SET_ROBOT_PARAMETER:SetRobotParameters(),
            MessageID.ID_SET_VEL:RobotVel(),
            MessageID.ID_GET_ODOM:RobotOdom(),
            MessageID.ID_GET_PID_DEBUG: RobotPIDData(),
            MessageID.ID_GET_IMU: RobotIMU(),
            MessageID.ID_GET_ENCODER_COUNT: RobotEncoderCount(),
            MessageID.ID_SET_MOTOR_PWM: RobotMotorPWM(),
            }

