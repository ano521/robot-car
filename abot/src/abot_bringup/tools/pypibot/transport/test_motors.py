import platform
import sys
sys.path.append("..")
import pypibot
from pypibot import log
from transport import Transport
from dataholder import MessageID
import params
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
    
    pwm=[0]*4
    for i in range(len(sys.argv)-1):
        pwm[i] = int(sys.argv[i+1])
    
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

    DataHolder[MessageID.ID_SET_MOTOR_PWM].pwm = pwm

    p = mboard.request(MessageID.ID_SET_MOTOR_PWM)
    if p:
        log.info('set pwm success')
    else:
        log.error('set pwm err')
        quit(1)

    log.info("****************get encoder count*****************")
    while run_flag:
        robotEncoder = DataHolder[MessageID.ID_GET_ENCODER_COUNT].encoder
        p = mboard.request(MessageID.ID_GET_ENCODER_COUNT)
        if p:
            log.info('encoder count: %s'%(('\t\t').join([str(x) for x in robotEncoder])))
        else:
            log.error('get encoder count err')
            quit(1)

        import time
        time.sleep(0.5)

    DataHolder[MessageID.ID_SET_MOTOR_PWM].pwm = [0]*4

    p = mboard.request(MessageID.ID_SET_MOTOR_PWM)
    if p:
        log.info('set pwm success')
    else:
        log.error('set pwm err')
        quit(1)