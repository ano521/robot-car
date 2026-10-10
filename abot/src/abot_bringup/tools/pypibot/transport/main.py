import platform
import sys
sys.path.append("..")
import pypibot
from pypibot import log
from transport import Transport
from dataholder import MessageID
import params
import time
import signal 

#for linux
port="/dev/pibot"

#for windows
#port="com3"

pypibot.assistant.enableGlobalExcept()
#log.enableFileLog(log_dir + "ros_$(Date8)_$(filenumber2).log")
log.setLevel("i")

run_flag = True

def exit(signum, frame):
    global run_flag
    run_flag = False

if __name__ == '__main__':
    signal.signal(signal.SIGINT, exit)

    mboard = Transport(port, params.pibotBaud)
    if not mboard.start():
        log.error("can not open %s"%port)
        sys.exit()
    
    DataHolder = mboard.getDataHolder()

    for num in range(0,3):
        log.info("****************get robot version*****************")
        boardVersion = DataHolder[MessageID.ID_GET_VERSION]
        p = mboard.request(MessageID.ID_GET_VERSION)
        if p:
            log.info("firmware version:%s buildtime:%s\r\n"%(boardVersion.version.decode(), boardVersion.build_time.decode()))
            break
        else:
            log.error('read firmware version err\r\n')
            import time
            time.sleep(1)
            if num == 2:
                log.error('please check connection or baudrate\r\n')
                sys.exit()
                
    # get robot parameter
    robotParam = DataHolder[MessageID.ID_GET_ROBOT_PARAMETER]
    p = mboard.request(MessageID.ID_GET_ROBOT_PARAMETER)
    if p:
        log.info("model_type:%d wheel_diameter:%d wheel_track:%d encoder_resolution:%d" \
                 %(robotParam.param.model_type, \
                   robotParam.param.wheel_diameter, \
                   robotParam.param.wheel_track, \
                   robotParam.param.encoder_resolution
                   ))

        log.info("do_pid_interval:%d kp:%d ki:%d kd:%d ko:%d" \
                 %(robotParam.param.do_pid_interval, \
                   robotParam.param.kp, \
                   robotParam.param.ki, \
                   robotParam.param.kd, \
                   robotParam.param.ko))

        log.info("cmd_last_time:%d imu_type:%d" \
                 %(robotParam.param.cmd_last_time,\
                   robotParam.param.imu_type
                   ))

        log.info("max_v:%d %d %d\r\n" \
                 %(robotParam.param.max_v_liner_x,\
                   robotParam.param.max_v_liner_y, \
                   robotParam.param.max_v_angular_z
                   ))
                   
        log.info("motor flag:%d encoder flag: %d\r\n" \
                 %(robotParam.param.motor_nonexchange_flag,\
                   robotParam.param.encoder_nonexchange_flag
                   ))
    else:
        log.error('get params err\r\n')
        quit(1)

    log.info("****************get odom&imu*****************")
    while run_flag:
        robotOdom = DataHolder[MessageID.ID_GET_ODOM]
        p = mboard.request(MessageID.ID_GET_ODOM)
        if p:
            log.info('request get odom success, vx=%d vy=%d vangular=%d,    x=%d y=%d yaw=%d'%(robotOdom.v_liner_x, \
                                                        robotOdom.v_liner_y, \
                                                        robotOdom.v_angular_z, \
                                                        robotOdom.x, \
                                                        robotOdom.y, \
                                                        robotOdom.yaw))
        else:
            log.error('get odom err')
            quit(1)
        
        robotIMU = DataHolder[MessageID.ID_GET_IMU].imu
        p = mboard.request(MessageID.ID_GET_IMU)
        if p:
            log.info('get imu success, imu=[%f %f %f    %f %f %f    %f %f %f]'%(robotIMU[0], robotIMU[1], robotIMU[2], \
                                                                                        robotIMU[3], robotIMU[4], robotIMU[5], \
                                                                                        robotIMU[6], robotIMU[7], robotIMU[8]))
        else:
            log.error('get imu err')
            quit(1)
       
        time.sleep(0.1) 
