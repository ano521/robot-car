import dataholder
import os
from dataholder import RobotImuType
from dataholder import RobotModelType

pibotModel = os.environ['PIBOT_MODEL']
boardType = os.environ['PIBOT_BOARD']
pibotBaud = os.environ['PIBOT_DRIVER_BAUDRATE']

print(pibotModel)
print(boardType)
print(pibotBaud)

pibotParam = dataholder.RobotParameters()

if pibotModel == "apollo" and boardType == "arduino":
    pibotParam = dataholder.RobotParameters(65, 175, 44, 10, \
                                                75, 2500, 0, 10, \
                                                250, 40, 0, 200, \
                                                RobotImuType.IMU_TYPE_GY85, 90, \
                                                RobotModelType.MODEL_TYPE_2WD_DIFF)
elif pibotModel == "apollo" and boardType == "stm32f1":
    pibotParam = dataholder.RobotParameters(65, 175, 44, 10, \
                                                320, 2700, 0, 10, \
                                                250, 50, 0, 200, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_2WD_DIFF)
elif pibotModel == "apollo" and boardType == "stm32f4":
    pibotParam = dataholder.RobotParameters(65, 175, 44, 10, \
                                                320, 2700, 0, 10, \
                                                250, 40, 0, 200, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_2WD_DIFF)
elif pibotModel == "zeus" and boardType == "stm32f4":
    pibotParam = dataholder.RobotParameters(58, 230, 44, 10, \
                                                320, 2700, 0, 10, \
                                                250, 50, 50, 250, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_3WD_OMNI)
elif pibotModel == "hades" and boardType == "stm32f4":
    pibotParam = dataholder.RobotParameters(76, 470, 44, 10, \
                                                320, 2700, 0, 10, \
                                                250, 50, 50, 250, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_4WD_MECANUM)
elif pibotModel == "hadesX" and boardType == "stm32f4":
    pibotParam = dataholder.RobotParameters(150, 565, 44, 10, \
                                                250, 2750, 0, 10, \
                                                250, 50, 50, 250, \
                                                RobotImuType.IMU_TYPE_GY87, 72, \
                                                RobotModelType.MODEL_TYPE_4WD_MECANUM)
elif pibotModel == "hera" and boardType == "stm32f4":
    pibotParam = dataholder.RobotParameters(82, 338, 44, 10, \
                                                320, 2700, 0, 10, \
                                                250, 50, 50, 250, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_4WD_DIFF)
elif pibotModel == "apolloX" and boardType == "arduino":
    pibotParam = dataholder.RobotParameters(96, 350, 68, 10, \
                                                75, 2500, 0, 10, \
                                                250, 40, 0, 200, \
                                                RobotImuType.IMU_TYPE_GY85, 90, \
                                                RobotModelType.MODEL_TYPE_2WD_DIFF)
elif pibotModel == "apolloX" and boardType == "stm32f1":
    pibotParam = dataholder.RobotParameters(96, 350, 68, 10, \
                                                250, 1200, 0, 10, \
                                                250, 50, 0, 200, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_2WD_DIFF)
elif pibotModel == "apolloX" and boardType == "stm32f4":
    pibotParam = dataholder.RobotParameters(96, 350, 68, 10, \
                                                250, 1200, 0, 10, \
                                                250, 50, 0, 200, \
                                                RobotImuType.IMU_TYPE_GY87, 90, \
                                                RobotModelType.MODEL_TYPE_2WD_DIFF)