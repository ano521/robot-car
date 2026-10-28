#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""c.py —— 标定工具的短命令版（真正的实现在 src/yolov8_ros/script/grasp_calib.py）

用法（在 /media/abot/999/abot 下）：

    python3 c.py list                看已记了几个点、当前残差
    python3 c.py click               开窗口，鼠标点一下 -> 打印像素 (u,v)
    python3 c.py add 312 240 0 200   记一条：像素(312,240) <-> 手臂(0,200) mm
    python3 c.py fit                 拟合，看残差
    python3 c.py fit --save          残差合格就写入 config/grasp_calib.json
    python3 c.py del 3               删掉第 3 条

流程：
    1) 把木块放在手臂能到的位置，用 a.py g x y 10 把爪子移到它正上方
    2) python3 c.py click        点画面上木块中心 -> 记住 (u,v)
    3) python3 c.py add u v x y  记下来（x y 就是你刚 g 命令里给的那两个数）
    4) 换位置重复 2~3，凑 6 个点（散布开，别都在一条线上）
    5) python3 c.py fit          残差 <= 10mm 才 --save
"""
import os
import runpy
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_TARGET = os.path.join(_HERE, "src", "yolov8_ros", "script", "grasp_calib.py")

if not os.path.exists(_TARGET):
    sys.exit("找不到 %s —— U 盘没插好，或者这是别的目录" % _TARGET)

args = sys.argv[1:]
# 允许简写：click -> --click，add -> --add ...
if args and not args[0].startswith("-"):
    args[0] = "--" + args[0]

sys.argv = [_TARGET] + args
runpy.run_path(_TARGET, run_name="__main__")
