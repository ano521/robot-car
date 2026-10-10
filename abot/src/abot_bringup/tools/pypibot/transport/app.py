# coding:utf-8

import sys
sys.path.append("..")
from pypibot import log
import pypibot

import params
import dataholder
from dataholder import MessageID
from transport import Transport
from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.QtCore import QObject,pyqtSignal
import pb
import threading

port = "/dev/pibot"

pypibot.assistant.enableGlobalExcept()
# log.enableFileLog(log_dir + "ros_$(Date8)_$(filenumber2).log")
log.setLevel("i")


class MainDialog(QDialog):
    encoder_signal = pyqtSignal(list)
    imu_signal = pyqtSignal(list)
    pid_debug_signal = pyqtSignal(list)

    def __init__(self, parent=None):
        super(QDialog, self).__init__(parent)
        self.ui = pb.Ui_pb()
        self.ui.setupUi(self)
        self.model_type_list = {"2wd-diff": dataholder.RobotModelType.MODEL_TYPE_2WD_DIFF,
                                "4wd-diff": dataholder.RobotModelType.MODEL_TYPE_4WD_DIFF,
                                "3wd-omni": dataholder.RobotModelType.MODEL_TYPE_3WD_OMNI,
                                "4wd-omni": dataholder.RobotModelType.MODEL_TYPE_4WD_OMNI,
                                "4wd-mecanum": dataholder.RobotModelType.MODEL_TYPE_4WD_MECANUM}

        self.model_value_list = {0: dataholder.RobotModelType.MODEL_TYPE_2WD_DIFF,
                                1: dataholder.RobotModelType.MODEL_TYPE_4WD_DIFF,
                                2: dataholder.RobotModelType.MODEL_TYPE_3WD_OMNI,
                                3: dataholder.RobotModelType.MODEL_TYPE_4WD_OMNI,
                                3: dataholder.RobotModelType.MODEL_TYPE_4WD_MECANUM}
        
        self.model_index_list = {dataholder.RobotModelType.MODEL_TYPE_2WD_DIFF: 0,
                                 dataholder.RobotModelType.MODEL_TYPE_4WD_DIFF: 1,
                                 dataholder.RobotModelType.MODEL_TYPE_3WD_OMNI: 2,
                                 dataholder.RobotModelType.MODEL_TYPE_4WD_OMNI: 3,
                                 dataholder.RobotModelType.MODEL_TYPE_4WD_MECANUM: 4}

        self.imu_type_list = {"gy65": 0, "gy85": 1, "gy87": 2}
        self.imu_value_list = {49: "gy65", 69: "gy85", 71: "gy87"}
        self.imu_index_list = {0: 49, 1: 69, 2: 71}

        self.init_ui()
        self.mboard = None

        self.init_dev()

        self.encoder_signal.connect(self.update_encoder)
        self.imu_signal.connect(self.update_imu)
        self.pid_debug_signal.connect(self.update_pid_debug)

        self._KeepRunning = True
        self.encoder_thread = threading.Thread(target=self._read_encoder)
        self.encoder_thread.start()

    def closeEvent(self, event):
        self._KeepRunning = False

    def init_ui(self):
        for model_type in self.model_type_list.keys():
            self.ui.combox_model.addItem(model_type)

        for imu_type in self.imu_type_list.keys():
            self.ui.combox_imu_type.addItem(imu_type)

        self.ui.slider_wheel_diameter.setMinimum(10)
        self.ui.slider_wheel_diameter.setMaximum(500)

        self.ui.slider_wheel_track.setMinimum(50)
        self.ui.slider_wheel_track.setMaximum(1000)

        self.ui.slider_encoder.setMinimum(1)
        self.ui.slider_encoder.setMaximum(500)

        self.ui.slider_motor_ratio.setMinimum(1)
        self.ui.slider_motor_ratio.setMaximum(1000)

        self.ui.slider_pid_interval.setMinimum(1)
        self.ui.slider_pid_interval.setMaximum(80)
        self.ui.slider_kp.setMinimum(0)
        self.ui.slider_kp.setMaximum(10000)
        self.ui.slider_ki.setMinimum(0)
        self.ui.slider_ki.setMaximum(32000)
        self.ui.slider_kd.setMinimum(0)
        self.ui.slider_kd.setMaximum(1000)
        self.ui.slider_ko.setMinimum(0)
        self.ui.slider_ko.setMaximum(1000)

        self.ui.slider_cmd_lasttime.setMinimum(0)
        self.ui.slider_cmd_lasttime.setMaximum(1000)
        self.ui.slider_vx_max.setMinimum(0)
        self.ui.slider_vx_max.setMaximum(500)
        self.ui.slider_vy_max.setMinimum(0)
        self.ui.slider_vy_max.setMaximum(500)
        self.ui.slider_va_max.setMinimum(0)
        self.ui.slider_va_max.setMaximum(2000)

        self.ui.comboBox_support_model.setVisible(False)
        self.ui.pushButton_load.setVisible(False)

        self.ui.pushButton_read.clicked.connect(self.read_params)
        self.ui.pushButton_set.clicked.connect(self.write_params)
        self.ui.pushButton_start.clicked.connect(self.start_motor)
        self.ui.pushButton_stop.clicked.connect(self.stop_motor)

        self.ui.pushButton_start_2.clicked.connect(self.start_control)
        self.ui.pushButton_stop_2.clicked.connect(self.stop_control)

        self.ui.slider_set_pwm1.setMinimum(-5000)
        self.ui.slider_set_pwm1.setMaximum(5000)
        self.ui.slider_set_pwm2.setMinimum(-5000)
        self.ui.slider_set_pwm2.setMaximum(5000)
        self.ui.slider_set_pwm3.setMinimum(-5000)
        self.ui.slider_set_pwm3.setMaximum(5000)
        self.ui.slider_set_pwm4.setMinimum(-5000)
        self.ui.slider_set_pwm4.setMaximum(5000)
        self.ui.tabWidget.setTabText(0, '1.参数配置')
        self.ui.tabWidget.setTabText(1, '2.电机测试')
        self.ui.tabWidget.setCurrentIndex(0)

    def update_param(self, param):
        log.i("type:%d %d" % (param.model_type, param.imu_type))
        try:
            self.ui.combox_model.setCurrentIndex(self.model_index_list[param.model_type])
        except Exception as e:
            print("model type err")

        try:
            self.ui.combox_imu_type.setCurrentIndex(
                self.imu_type_list[self.imu_value_list[param.imu_type]])
        except Exception as e:
            print("imu type err")

        try:
            self.ui.slider_wheel_diameter.setSliderPosition(
                param.wheel_diameter)
            self.ui.slider_wheel_track.setSliderPosition(param.wheel_track)
            self.ui.slider_encoder.setSliderPosition(param.encoder_resolution)
            self.ui.slider_motor_ratio.setSliderPosition(param.motor_ratio)

            self.ui.checkBox_motor1.setChecked(
                param.motor_nonexchange_flag & 0x1)
            self.ui.checkBox_motor2.setChecked(
                param.motor_nonexchange_flag & 0x2)
            self.ui.checkBox_motor3.setChecked(
                param.motor_nonexchange_flag & 0x4)
            self.ui.checkBox_motor4.setChecked(
                param.motor_nonexchange_flag & 0x8)

            self.ui.checkBox_encoder1.setChecked(
                param.encoder_nonexchange_flag & 0x1)
            self.ui.checkBox_encoder2.setChecked(
                param.encoder_nonexchange_flag & 0x2)
            self.ui.checkBox_encoder3.setChecked(
                param.encoder_nonexchange_flag & 0x4)
            self.ui.checkBox_encoder4.setChecked(
                param.encoder_nonexchange_flag & 0x8)
        except Exception as e:
            print("motor dir param err")

        try:
            self.ui.slider_cmd_lasttime.setSliderPosition(param.cmd_last_time)
            self.ui.slider_vx_max.setSliderPosition(param.max_v_liner_x)
            self.ui.slider_vy_max.setSliderPosition(param.max_v_liner_y)
            self.ui.slider_va_max.setSliderPosition(param.max_v_angular_z)
        except Exception as e:
            print("pid param err")

        try:
            self.ui.slider_pid_interval.setSliderPosition(
                param.do_pid_interval)
            self.ui.slider_kp.setSliderPosition(param.kp)
            self.ui.slider_ki.setSliderPosition(param.ki)
            self.ui.slider_kd.setSliderPosition(param.kd)
            self.ui.slider_ko.setSliderPosition(param.ko)
        except Exception as e:
            print("velocity limit param err")

    def read_params(self):
        # get robot parameter
        robotParam = self.DataHolder[MessageID.ID_GET_ROBOT_PARAMETER]
        p = self.mboard.request(MessageID.ID_GET_ROBOT_PARAMETER)
        if p:
            log.info("model_type:%d wheel_diameter:%d wheel_track:%d encoder_resolution:%d"
                     % (robotParam.param.model_type,
                        robotParam.param.wheel_diameter,
                        robotParam.param.wheel_track,
                        robotParam.param.encoder_resolution
                        ))

            log.info("do_pid_interval:%d kp:%d ki:%d kd:%d ko:%d"
                     % (robotParam.param.do_pid_interval,
                        robotParam.param.kp,
                        robotParam.param.ki,
                        robotParam.param.kd,
                        robotParam.param.ko))

            log.info("cmd_last_time:%d imu_type:%d"
                     % (robotParam.param.cmd_last_time,
                        robotParam.param.imu_type
                        ))

            log.info("max_v:%d %d %d"
                     % (robotParam.param.max_v_liner_x,
                        robotParam.param.max_v_liner_y,
                        robotParam.param.max_v_angular_z
                        ))

            log.info("motor flag:%d encoder flag: %d"
                     % (robotParam.param.motor_nonexchange_flag,
                        robotParam.param.encoder_nonexchange_flag
                        ))
        else:
            log.error('get params err')
            return False

        self.update_param(robotParam.param)

    def get_input_param(self):
        params = dataholder.RobotParameters()

        params.wheel_diameter = self.ui.slider_wheel_diameter.sliderPosition()
        params.wheel_track = self.ui.slider_wheel_track.sliderPosition()
        params.encoder_resolution = self.ui.slider_encoder.sliderPosition()
        params.do_pid_interval = self.ui.slider_pid_interval.sliderPosition()
        params.kp = self.ui.slider_kp.sliderPosition()
        params.ki = self.ui.slider_ki.sliderPosition()
        params.kd = self.ui.slider_kd.sliderPosition()
        params.ko = self.ui.slider_ko.sliderPosition()
        params.cmd_last_time = self.ui.slider_cmd_lasttime.sliderPosition()
        params.max_v_liner_x = self.ui.slider_vx_max.sliderPosition()
        params.max_v_liner_y = self.ui.slider_vy_max.sliderPosition()
        params.max_v_angular_z = self.ui.slider_va_max.sliderPosition()
        params.motor_ratio = self.ui.slider_motor_ratio.sliderPosition()
        params.imu_type = self.imu_index_list[self.ui.combox_imu_type.currentIndex(
        )]

        params.model_type = self.model_value_list[self.ui.combox_model.currentIndex()]

        params.motor_nonexchange_flag = 0

        if self.ui.checkBox_motor1.isChecked():
            params.motor_nonexchange_flag = params.motor_nonexchange_flag | 0x1
        else:
            params.motor_nonexchange_flag = params.motor_nonexchange_flag & 0xfe

        if self.ui.checkBox_motor2.isChecked():
            params.motor_nonexchange_flag = params.motor_nonexchange_flag | 0x2
        else:
            params.motor_nonexchange_flag = params.motor_nonexchange_flag & 0xfd

        if self.ui.checkBox_motor3.isChecked():
            params.motor_nonexchange_flag = params.motor_nonexchange_flag | 0x4
        else:
            params.motor_nonexchange_flag = params.motor_nonexchange_flag & 0xfb

        if self.ui.checkBox_motor4.isChecked():
            params.motor_nonexchange_flag = params.motor_nonexchange_flag | 0x8
        else:
            params.motor_nonexchange_flag = params.motor_nonexchange_flag & 0xf7

        params.encoder_nonexchange_flag = 0

        if self.ui.checkBox_encoder1.isChecked():
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag | 0x1
        else:
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag & 0xfe

        if self.ui.checkBox_encoder2.isChecked():
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag | 0x2
        else:
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag & 0xfd

        if self.ui.checkBox_encoder3.isChecked():
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag | 0x4
        else:
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag & 0xfb

        if self.ui.checkBox_encoder4.isChecked():
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag | 0x8
        else:
            params.encoder_nonexchange_flag = params.encoder_nonexchange_flag & 0xf7

        return params

    def write_params(self):
        self.DataHolder[MessageID.ID_SET_ROBOT_PARAMETER].param = self.get_input_param()
        p = self.mboard.request(MessageID.ID_SET_ROBOT_PARAMETER)
        if p:
            log.info('set parameter success')
        else:
            log.error('set parameter err')
            quit(1)
    
    def update_pid_debug(self, pid_data):
        self.ui.label_input_1.setText(str(pid_data[0]))
        self.ui.label_input_2.setText(str(pid_data[1]))
        self.ui.label_input_3.setText(str(pid_data[2]))
        self.ui.label_input_4.setText(str(pid_data[3]))
        self.ui.label_output_1.setText(str(pid_data[4]))
        self.ui.label_output_2.setText(str(pid_data[5]))
        self.ui.label_output_3.setText(str(pid_data[6]))
        self.ui.label_output_4.setText(str(pid_data[7]))

    def update_imu(self, imu):
        log.info('imu: %s'%(('\t\t').join([str(x) for x in imu])))
        self.ui.label_acc_x.setText(str(round(imu[0], 6)))
        self.ui.label_acc_y.setText(str(round(imu[1], 6)))
        self.ui.label_acc_z.setText(str(round(imu[2], 6)))
        self.ui.label_gyro_x.setText(str(round(imu[3], 6)))
        self.ui.label_gyro_y.setText(str(round(imu[4], 6)))
        self.ui.label_gyro_z.setText(str(round(imu[5], 6)))
        self.ui.label_magn_x.setText(str(round(imu[6], 6)))
        self.ui.label_magn_y.setText(str(round(imu[7], 6)))
        self.ui.label_magn_z.setText(str(round(imu[8], 6)))

    def update_encoder(self, encoder):
        log.debug('encoder count: %s'%(('\t\t').join([str(x) for x in encoder])))
        self.ui.label_feedback1.setText(str(encoder[0]))
        self.ui.label_feedback2.setText(str(encoder[1]))
        self.ui.label_feedback3.setText(str(encoder[2]))
        self.ui.label_feedback4.setText(str(encoder[3]))


    def _read_encoder(self):
        while self._KeepRunning:
            robot_encoder = self.DataHolder[MessageID.ID_GET_ENCODER_COUNT].encoder
            p = self.mboard.request(MessageID.ID_GET_ENCODER_COUNT)
            if p:
                # log.info('encoder count: %s'%(('\t\t').join([str(x) for x in robot_encoder])))
                self.encoder_signal.emit([int(x) for x in robot_encoder])
            
            robot_imu = self.DataHolder[MessageID.ID_GET_IMU].imu
            p = self.mboard.request(MessageID.ID_GET_IMU)
            if p:
                # log.info('imu: %s'%(('\t\t').join([str(x) for x in robot_imu])))
                self.imu_signal.emit([x for x in robot_imu])

            pid_data = self.DataHolder[MessageID.ID_GET_PID_DEBUG].pid_data
            p = self.mboard.request(MessageID.ID_GET_PID_DEBUG)
            if p:
                # log.info('imu: %s'%(('\t\t').join([str(x) for x in robot_imu])))
                self.pid_debug_signal.emit([x for x in pid_data])
            import time
            time.sleep(0.1)

    def start_motor(self):
        self.DataHolder[MessageID.ID_SET_MOTOR_PWM].pwm = [self.ui.slider_set_pwm1.sliderPosition(),
                                                            self.ui.slider_set_pwm2.sliderPosition(),
                                                            self.ui.slider_set_pwm3.sliderPosition(),
                                                            self.ui.slider_set_pwm4.sliderPosition()]
        p = self.mboard.request(MessageID.ID_SET_MOTOR_PWM)
        if p:
            log.info('set pwm success')
        else:
            log.error('set pwm err')

    def stop_motor(self):
        self.DataHolder[MessageID.ID_SET_MOTOR_PWM].pwm = [0]*4
        p = self.mboard.request(MessageID.ID_SET_MOTOR_PWM)
        if p:
            log.info('set pwm success')
        else:
            log.error('set pwm err')
        
    def start_control(self):
        self.DataHolder[MessageID.ID_SET_VEL].v_liner_x = 200
        self.DataHolder[MessageID.ID_SET_VEL].v_liner_y = 0
        self.DataHolder[MessageID.ID_SET_VEL].v_angular_z = 10
        p = self.mboard.request(MessageID.ID_SET_VEL)
        if p:
            log.info('set vel success')
        else:
            log.error('set vel err')

    def stop_control(self):
        self.DataHolder[MessageID.ID_SET_VEL].v_liner_x = 0
        self.DataHolder[MessageID.ID_SET_VEL].v_liner_y = 0
        self.DataHolder[MessageID.ID_SET_VEL].v_angular_z = 0
        p = self.mboard.request(MessageID.ID_SET_VEL)
        if p:
            log.info('set vel success')
        else:
            log.error('set vel err')
        
    def init_dev(self):
        self.mboard = Transport(port, params.pibotBaud)
        if not self.mboard.start():
            log.error("can not open %s" % port)
            return False

        self.DataHolder = self.mboard.getDataHolder()

        for num in range(0, 3):
            log.info("****************get robot version*****************")
            boardVersion = self.DataHolder[MessageID.ID_GET_VERSION]
            p = self.mboard.request(MessageID.ID_GET_VERSION)
            if p:
                log.info("firmware version:%s buildtime:%s" % (
                    boardVersion.version.decode(), boardVersion.build_time.decode()))
                break
            else:
                log.error('read firmware version err')
                import time
                time.sleep(1)
                if num == 2:
                    log.error('please check connection or baudrate')
                    return False

        return self.read_params()

if __name__ == '__main__':
    app = QApplication(sys.argv)

    import qdarkstyle
    app.setStyleSheet(qdarkstyle.load_stylesheet(qt_api='pyqt5'))

    # from qt_material import apply_stylesheet
    # apply_stylesheet(app, theme='light_teal.xml')
    myDlg = MainDialog()
    myDlg.show()
    sys.exit(app.exec_())
