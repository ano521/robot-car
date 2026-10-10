# 机械臂键盘控制 - rqt_reconfigure 动态参数配置

这个项目实现了基于 rqt_reconfigure 的机械臂动态控制功能。

## 功能特性

### 可动态调整的参数：
1. **位置控制**：
   - `kms_x`: X坐标 (-300mm 到 300mm)
   - `kms_y`: Y坐标 (50mm 到 400mm) 
   - `kms_z`: Z高度 (0mm 到 200mm)

2. **舵机控制**：
   - `servo_yuntai`: 云台舵机PWM值 (500-2500)
   - `servo_zhuazi`: 爪子舵机PWM值 (500-2500)

3. **运动参数**：
   - `move_time`: 运动时间 (500-5000ms)

4. **动作触发器**：
   - `trigger_action`: 触发抓取动作
   - `reset_position`: 重置到初始位置
   - `gripper_open`: 张开爪子
   - `gripper_close`: 闭合爪子

5. **调试模式**：
   - `debug_mode`: 开启调试信息输出

## 使用方法

### 1. 编译项目
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动机械臂控制节点
```bash
# 方法1: 使用launch文件启动
roslaunch yolov8_ros keyboard_grasp.launch

# 方法2: 直接运行脚本
rosrun yolov8_ros keyboard_grasp.py
```

### 3. 打开动态参数配置界面
```bash
# 启动rqt_reconfigure GUI
rosrun rqt_reconfigure rqt_reconfigure
```

### 4. 控制机械臂
1. 在rqt_reconfigure界面中找到 `keyboard_grasp_node`
2. 调整位置参数 (kms_x, kms_y, kms_z)
3. 点击 `trigger_action` 触发抓取动作
4. 使用 `gripper_open/close` 控制爪子
5. 使用 `reset_position` 重置机械臂位置

## 注意事项

1. **安全范围**：请确保设置的坐标在机械臂的安全工作范围内
2. **串口连接**：确保机械臂已正确连接到 `/dev/arm` 串口
3. **实时调试**：开启 `debug_mode` 可以看到详细的参数变化信息
4. **PWM范围**：舵机PWM值建议保持在安全范围内（1000-2000）

## 故障排除

1. **连接问题**：检查串口连接和权限
2. **参数不响应**：确认ROS节点正常运行
3. **动作异常**：检查运动学参数设置

## 技术说明

- 使用 `dynamic_reconfigure` 实现实时参数调整
- 集成了逆运动学计算 (`z_kinematics`)
- 支持多线程安全的机械臂控制
- 提供了直观的图形化参数调整界面