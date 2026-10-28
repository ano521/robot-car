#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐点验证已测导航点。

这个程序只发送 move_base 导航目标；不会启动抓取、机械臂，也不会修改
move_base 的代价地图参数。运行前必须由操作者在 RViz 中完成 2D Pose Estimate。
"""

import argparse
import csv
import math
import os
import sys
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_conversions import transformations


DEFAULT_TIMEOUT = 120.0
DEFAULT_POSITION_TOLERANCE = 0.10
# 与 DWAPlannerROS 的 yaw_goal_tolerance: 0.1 rad（约 5.7°）一致，取整为 6°。
DEFAULT_YAW_TOLERANCE_DEG = 6.0
DEFAULT_STABLE_SECONDS = 2.0
DEFAULT_STABLE_WAIT_SECONDS = 10.0
DEFAULT_LINEAR_LIMIT = 0.02
DEFAULT_ANGULAR_LIMIT = 0.05
ODOM_TOPIC = '/wheel_odom'


def project_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def load_points(csv_path):
    points = {}
    with open(csv_path, newline='') as handle:
        for row in csv.reader(handle):
            if not row or row[0].lstrip().startswith('#'):
                continue
            if len(row) < 5:
                raise ValueError('点位行列数不足：{}'.format(','.join(row)))
            index = int(row[0])
            points[index] = {
                'id': index,
                'name': row[1].strip() or 'P{}'.format(index),
                'x': float(row[2]),
                'y': float(row[3]),
                'yaw': float(row[4]),
            }
    if not points:
        raise ValueError('点位文件中没有可用点位：{}'.format(csv_path))
    return points


def yaw_from_quaternion(quaternion):
    return math.degrees(transformations.euler_from_quaternion([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    ])[2])


def angle_error_degrees(actual, expected):
    return abs((actual - expected + 180.0) % 360.0 - 180.0)


def pose_from_message(message):
    pose = message.pose.pose
    return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


def wait_for_valid_scan(timeout):
    scan = rospy.wait_for_message('/scan', LaserScan, timeout=timeout)
    valid = [item for item in scan.ranges
             if math.isfinite(item) and scan.range_min <= item <= scan.range_max]
    if not valid:
        raise RuntimeError('/scan 没有有效量程数据，拒绝发送导航目标')
    rospy.loginfo('/scan 正常：%d 个有效量程', len(valid))


def wait_for_odom(timeout):
    rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=timeout)
    rospy.loginfo('%s 正常：已收到里程计消息', ODOM_TOPIC)


def wait_until_stable(stable_seconds, max_wait_seconds, linear_limit, angular_limit):
    """等待连续 stable_seconds 的新鲜、低速里程计；超时才判为未停稳。"""
    latest = {'linear': None, 'angular': None, 'received_at': None}

    def callback(message):
        twist = message.twist.twist
        latest['linear'] = math.hypot(twist.linear.x, twist.linear.y)
        latest['angular'] = abs(twist.angular.z)
        latest['received_at'] = time.monotonic()

    subscriber = rospy.Subscriber(ODOM_TOPIC, Odometry, callback, queue_size=50)
    deadline = time.monotonic() + max_wait_seconds
    stable_since = None
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        now = time.monotonic()
        message_is_fresh = (
            latest['received_at'] is not None and now - latest['received_at'] <= 0.5
        )
        is_stable = (
            message_is_fresh and latest['linear'] <= linear_limit and
            latest['angular'] <= angular_limit
        )
        if is_stable:
            if stable_since is None:
                stable_since = now
            if now - stable_since >= stable_seconds:
                detail = ('连续 %.1f 秒稳定（线速度 %.3f m/s、角速度 %.3f rad/s）' % (
                    stable_seconds, latest['linear'], latest['angular']))
                subscriber.unregister()
                return True, detail
        else:
            stable_since = None
        rate.sleep()
    subscriber.unregister()

    if latest['received_at'] is None:
        return False, '没有收到 {}，不能确认已停稳'.format(ODOM_TOPIC)
    return False, ('等待 %.1f 秒仍未连续稳定 %.1f 秒；最后线速度 %.3f m/s、角速度 %.3f rad/s' % (
        max_wait_seconds, stable_seconds, latest['linear'], latest['angular']))


def nearest_order(points, selected, start_x, start_y):
    pending = [points[index] for index in selected]
    ordered = []
    current_x, current_y = start_x, start_y
    while pending:
        next_point = min(
            pending,
            key=lambda item: math.hypot(item['x'] - current_x, item['y'] - current_y)
        )
        pending.remove(next_point)
        ordered.append(next_point)
        current_x, current_y = next_point['x'], next_point['y']
    return ordered


def build_goal(point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point['x']
    goal.target_pose.pose.position.y = point['y']
    quaternion = transformations.quaternion_from_euler(
        0.0, 0.0, math.radians(point['yaw'])
    )
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    return goal


def check_arrival(point, position_tolerance, yaw_tolerance):
    pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5.0)
    actual_x, actual_y, actual_yaw = pose_from_message(pose)
    distance = math.hypot(actual_x - point['x'], actual_y - point['y'])
    yaw_error = angle_error_degrees(actual_yaw, point['yaw'])
    passed = distance <= position_tolerance and yaw_error <= yaw_tolerance
    detail = ('实际 (%.3f, %.3f, %.1f°)，误差 %.3f m / %.1f°' %
              (actual_x, actual_y, actual_yaw, distance, yaw_error))
    return passed, detail


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='按 P0 到 P9 的固定任务顺序逐个验证 points_measured.csv 中的导航点。'
    )
    parser.add_argument('--run', action='store_true',
                        help='确认实际发送目标；省略时仅显示计划，不会移动机器人。')
    parser.add_argument('--point', type=int, action='append', dest='points',
                        help='只验证指定编号；可重复，例如 --point 2 --point 5。')
    parser.add_argument('--nearest-order', action='store_true',
                        help='显式要求按当前 AMCL 位置就近排序；默认严格按 P0 到 P9 执行。')
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT,
                        help='单点导航超时秒数，默认 %(default)s。')
    parser.add_argument('--position-tolerance', type=float,
                        default=DEFAULT_POSITION_TOLERANCE,
                        help='到位距离验收阈值（米），默认 %(default)s。')
    parser.add_argument('--yaw-tolerance', type=float,
                        default=DEFAULT_YAW_TOLERANCE_DEG,
                        help='朝向验收阈值（度），默认 %(default)s。')
    parser.add_argument('--stable-wait', type=float, default=DEFAULT_STABLE_WAIT_SECONDS,
                        help='到点后等待停稳的最长秒数，默认 %(default)s。')
    parser.add_argument('--pause-between-points', action='store_true',
                        help='每点通过后等待人工 Enter；默认自动继续下一点。')
    return parser.parse_args()


def main():
    args = parse_arguments()
    csv_path = os.path.join(project_root(), 'points_measured.csv')
    points = load_points(csv_path)
    selected = args.points or sorted(points)
    unknown = [index for index in selected if index not in points]
    if unknown:
        raise ValueError('不存在的点位编号：{}'.format(', '.join(map(str, unknown))))
    if len(set(selected)) != len(selected):
        raise ValueError('--point 不能重复')

    rospy.init_node('validate_measured_points', anonymous=True)
    rospy.loginfo('安全前提：已在 RViz 完成 2D Pose Estimate，场地无人且急停可用。')
    wait_for_valid_scan(10.0)
    wait_for_odom(10.0)
    start_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=10.0)
    start_x, start_y, start_yaw = pose_from_message(start_pose)

    if args.nearest_order:
        route = nearest_order(points, selected, start_x, start_y)
    else:
        route = [points[index] for index in selected]

    print('\n当前 AMCL：x=%.3f, y=%.3f, yaw=%.1f°' % (start_x, start_y, start_yaw))
    print('本次顺序：' + ' -> '.join('P{}'.format(item['id']) for item in route))
    for item in route:
        print('  P{0}: ({1:.4f}, {2:.4f}, {3:.2f}°)'.format(
            item['id'], item['x'], item['y'], item['yaw']))

    if not args.run:
        print('\n这是预览，未发送任何目标。确认后执行：')
        print('  bash ./6-validate-points.sh --run')
        return 0

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(20.0)):
        raise RuntimeError('等待 move_base 超时，拒绝执行')

    for ordinal, point in enumerate(route, 1):
        print('\n[{}/{}] 发送 P{}；Ctrl-C 会取消当前目标。'.format(
            ordinal, len(route), point['id']))
        client.send_goal(build_goal(point))
        if not client.wait_for_result(rospy.Duration(args.timeout)):
            client.cancel_goal()
            raise RuntimeError('P{} 在 {:.0f} 秒内未完成，已取消，停止后续点位'.format(
                point['id'], args.timeout))
        state = client.get_state()
        if state != GoalStatus.SUCCEEDED:
            raise RuntimeError('P{} 导航失败，move_base 状态码 {}，停止后续点位'.format(
                point['id'], state))

        arrived, arrived_detail = check_arrival(
            point, args.position_tolerance, args.yaw_tolerance
        )
        stopped, stopped_detail = wait_until_stable(
            DEFAULT_STABLE_SECONDS, args.stable_wait,
            DEFAULT_LINEAR_LIMIT, DEFAULT_ANGULAR_LIMIT
        )
        print('  到位检查：' + arrived_detail)
        print('  停稳检查：' + stopped_detail)
        if not arrived or not stopped:
            raise RuntimeError('P{} 未通过验收，停止后续点位'.format(point['id']))

        if args.pause_between_points:
            answer = input('  P{} 通过。按 Enter 去下一点；输入 q 停止： '.format(
                point['id'])).strip().lower()
            if answer in ('q', 'quit', 'stop'):
                print('操作者已停止；没有发送下一点。')
                return 0
        else:
            print('  P{} 通过，自动继续下一点。'.format(point['id']))

    print('\n10 个点均已通过本次自动检查。仍须以现场肉眼确认结果为准。')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        print('\n收到中断，未继续发送目标。')
        sys.exit(130)
    except Exception as error:
        print('\n[失败] {}'.format(error), file=sys.stderr)
        sys.exit(1)
