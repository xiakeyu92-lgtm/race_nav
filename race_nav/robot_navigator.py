#!/usr/bin/env python3
"""
robot_navigator.py — 封装 Nav2 NavigateToPose Action，提供阻塞式导航接口。

======================== 使用说明 ========================
Gen1 (当前):  纯激光雷达导航，go_to() 阻塞等待到达
Gen2 扩展点:  在 go_to() 途中订阅相机话题，检测二维码/立牌
Gen3 扩展点:  在 go_to() 途中处理雷达避障，必要时取消当前 goal 重新规划
==========================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import sin, cos


class RobotNavigator(Node):
    """
    封装所有与 Nav2 的交互。
    Gen2 时: 在这里添加相机订阅和回调。
    Gen3 时: 在这里添加避障检测和重规划逻辑。
    """

    def __init__(self):
        super().__init__('robot_navigator')

        # ---------- Nav2 Action Client ----------
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None      # 当前执行的 goal handle，Gen3 避障取消用
        self._result_future = None    # 当前 goal 的 result future

        # ---------- Gen2 扩展点: 相机订阅 ----------
        # self.qr_sub = self.create_subscription(
        #     Image, '/camera/qr_topic', self._on_qr_detected, 10)
        # self.standee_sub = self.create_subscription(...)
        # self.detected_qr_value = None   # 途中识别到的二维码值
        # self.detected_standee = False   # 途中有没有检测到立牌

        # ---------- Gen3 扩展点: 雷达避障 ----------
        # self.obstacle_sub = self.create_subscription(
        #     LaserScan, '/scan', self._on_scan, 10)
        # self.obstacle_detected = False

        self.get_logger().info('RobotNavigator 初始化完成 ✅')

    # ==================== 初始定位 ====================

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0):
        """
        通过 /initialpose 话题设置 AMCL 初始位姿。
        在比赛开始时调用一次，告诉车"你在哪里"。

        Args:
            x, y:   地图坐标系下的位置 (m)
            yaw:    朝向角 (rad)
        """
        pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = sin(yaw / 2)
        msg.pose.pose.orientation.w = cos(yaw / 2)

        # 协方差设小 → "我比较确定我的位置" → AMCL 粒子收敛快
        msg.pose.covariance[0] = 0.25   # x 方差
        msg.pose.covariance[7] = 0.25   # y 方差
        msg.pose.covariance[35] = 0.0685  # yaw 方差

        self.get_logger().info(
            f'📍 发布初始位姿: ({x:.3f}, {y:.3f}, yaw={yaw:.3f})')

        # 多发几次确保 AMCL 收到（topic 不是 action，不能等确认）
        import time
        for i in range(5):
            pub.publish(msg)
            time.sleep(0.05)

    # ==================== 导航动作 ====================

    def go_to(self, x: float, y: float, yaw: float = 0.0,
              timeout_sec: float = 60.0) -> bool:
        """
        发送 NavigateToPose goal，阻塞等待结果。

        Args:
            x, y:        目标点坐标 (map 坐标系)
            yaw:         目标朝向 (rad), 0 = 朝 +x 方向
            timeout_sec: 超时时间

        Returns:
            True  → 到达目标
            False → 失败/超时/被拒

        ======================== Gen2 扩展说明 ========================
        在 go_to() 阻塞期间 (spin_until_future_complete)，ROS2 回调仍在
        运行。Gen2 时你需要做的:
        1. 在 go_to() 前注册相机回调 → 回调里设置 self.detected_qr_value
        2. go_to() 返回后检查 self.detected_qr_value
        3. 如果检测到二维码，即可解码 → 决定方向
        4. 检测到立牌 → 回调里调 display 节点显示图文
        ================================================================

        ======================== Gen3 扩展说明 ========================
        避障方案:
        1. 订阅 /scan 话题
        2. 在回调里检查前方是否有障碍物 (在一定角度和距离内)
        3. 检测到障碍物 → cancel_goal_async() → 等障碍物清除 → 重新 go_to()
        或者更高级: 在 BT 层面用 nav2_obstacle_avoidance 插件
        ================================================================
        """
        # 构造 goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.orientation.w = cos(yaw / 2)

        self.get_logger().info(
            f'🚗 发送导航目标: ({x:.3f}, {y:.3f}, yaw={yaw:.3f}), timeout={timeout_sec}s')

        # 等待 Action Server 就绪
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 Action Server 未就绪！检查 bt_navigator 是否启动')
            return False

        # 发送 goal
        send_future = self.nav_client.send_goal_async(
            NavigateToPose.Goal(pose=goal_msg))
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)

        self._goal_handle = send_future.result()

        if self._goal_handle is None:
            self.get_logger().error('❌ send_goal 超时')
            return False

        if not self._goal_handle.accepted:
            self.get_logger().warn('❌ Goal 被 Nav2 拒绝 (可能已有 goal 在执行)')
            return False

        self.get_logger().info('✅ Goal 已接受，开始执行...')

        # 等待结果
        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, self._result_future, timeout_sec=timeout_sec)

        # 检查是否拿到结果
        if not self._result_future.done():
            self.get_logger().warn(
                f'⏰ 导航超时 ({timeout_sec}s)，取消 goal...')
            self._goal_handle.cancel_goal_async()
            return False

        result = self._result_future.result()

        # Nav2 的 NavigateToPose.Result 里 result.status == 4 表示 SUCCEEDED
        # 参考: action_msgs/GoalStatus
        if result.status == 4:
            self.get_logger().info('✅ 到达目标点')
            return True
        else:
            status_map = {
                0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING',
                3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED',
                6: 'ABORTED'
            }
            status_name = status_map.get(result.status, f'???({result.status})')
            self.get_logger().warn(f'❌ 导航失败: status={result.status} ({status_name})')
            return False

    # ==================== 辅助函数 ====================

    def cancel_current_goal(self):
        """取消当前正在执行的导航 goal (Gen3 避障时用)"""
        if self._goal_handle is not None:
            self.get_logger().info('⚠️  取消当前导航 goal')
            self._goal_handle.cancel_goal_async()

    # ==================== Gen2 回调 (占位) ====================

    # def _on_qr_detected(self, msg):
    #     """相机检测到二维码时调用 (Gen2)"""
    #     self.detected_qr_value = decode_qr(msg)
    #     self.get_logger().info(f'📷 检测到二维码: {self.detected_qr_value}')
    #
    # def _on_standee_detected(self, msg):
    #     """相机检测到人形立牌时调用 (Gen2)"""
    #     self.detected_standee = True
    #     self.get_logger().info('🪧 检测到人形立牌！')
    #     # 调用大模型识别 + 显示到屏幕

    # ==================== Gen3 回调 (占位) ====================

    # def _on_scan(self, msg):
    #     """激光雷达回调 — 检测前方障碍物 (Gen3)"""
    #     # 检查正前方 ±30° 范围内 0.5m 内是否有障碍物
    #     front_angles = range(-30, 31)  # 共 61 个射线
    #     for i in front_angles:
    #         if msg.ranges[i] < 0.5 and msg.ranges[i] > msg.range_min:
    #             self.obstacle_detected = True
    #             self.cancel_current_goal()
    #             break
    #     else:
    #         self.obstacle_detected = False
