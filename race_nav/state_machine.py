#!/usr/bin/env python3
"""
state_machine.py — 比赛状态机 (主入口)。

======================== 状态流转图 ========================

    ┌─────────────────────────────────────────────────────┐
    │                                                     │
    ▼                                                     │
  INIT ──→ GO_TO_QR ──→ DECODE_QR ──→ DO_ELLIPSE ──→ RETURN_P ──→ FINISHED
              │              │             │               │
              └──────────────┴─────────────┴───────────────┘
                             任何状态都可能 → FAILED

======================== 各代功能对照 ========================
         Gen1 (当前)    Gen2 (加相机)        Gen3 (加避障)
INIT     set_pose       ← 同左              ← 同左
GO_TO_QR 纯导航        途中检测QR           途中检测QR + 避障
DECODE   qr_value=1    真正解码              ← 同左
ELLIPSE  纯航点导航    途中检测立牌+大模型   + 雷达避障
RETURN_P 导航回P       ← 同左              ← 同左
======================== 各代功能对照 ========================

启动方式:
    ros2 run race_nav race_state_machine
    或通过 launch 文件: ros2 launch race_nav race_bringup.launch.py
"""

import sys
import rclpy
from enum import Enum, auto

from race_nav.robot_navigator import RobotNavigator
from race_nav.waypoint_manager import WaypointManager


# ==================== 状态枚举 ====================

class RaceState(Enum):
    INIT         = auto()   # 初始定位
    GO_TO_QR     = auto()   # P 点 → 二维码任务点
    DECODE_QR    = auto()   # 解码二维码 (Gen1 写死)
    DO_ELLIPSE   = auto()   # 走椭圆轨道
    RETURN_P     = auto()   # 返回 P 点
    FINISHED     = auto()   # 🎉 完成
    FAILED       = auto()   # ❌ 失败


# ==================== 状态机 ====================

class RaceStateMachine:
    """
    比赛全流程状态机。

    Gen1: 方向写死，不识别二维码/立牌，不避障。
    只需改下面 4 个坐标 + 1 个方向即可跑通全流程。
    """

    # ================================================================
    # ⚠️  改这里！用 RViz2 Publish Point 获取这些坐标
    # ================================================================
    P_POINT       = (0.0,  0.0,  0.0)     # 发车点 P (x, y, yaw)
    QR_POINT      = (3.0,  0.0,  0.0)     # 二维码任务点 (x, y, yaw)

    # ================================================================
    # ⚠️  Gen1 写死方向: 'clockwise' 或 'counterclockwise'
    #     Gen2 会从二维码解码自动决定, 这里不再写死
    # ================================================================
    HARDCODED_DIRECTION = 'clockwise'

    # 每个 waypoint 的导航超时 (秒), 椭圆大的话调大
    WAYPOINT_TIMEOUT = 45.0

    def __init__(self):
        self.nav = RobotNavigator()
        self.wp = WaypointManager(self.HARDCODED_DIRECTION)
        self.state = RaceState.INIT

        # 统计
        self.lap_count = 0

    def run(self):
        """主循环: 一个状态一个状态往下走。"""
        logger = self.nav.get_logger()
        logger.info('=' * 50)
        logger.info('🏁 智能车竞赛 全流程自主导航')
        logger.info(f'   Gen1: 激光雷达 only | 方向={self.HARDCODED_DIRECTION}')
        logger.info(f'   P点={self.P_POINT} | QR点={self.QR_POINT}')
        logger.info('=' * 50)

        while rclpy.ok() and self.state not in (
                RaceState.FINISHED, RaceState.FAILED):
            self._step()

        # 最终汇报
        if self.state == RaceState.FINISHED:
            logger.info('🎉🎉🎉 比赛流程全部完成！ 🎉🎉🎉')
        elif self.state == RaceState.FAILED:
            logger.error('❌ 比赛流程失败，检查日志')
        else:
            logger.info('ROS2 已关闭，状态机退出')

    # ==================== 单步状态处理 ====================

    def _step(self):
        """根据当前状态执行对应动作并切换状态。"""
        log = self.nav.get_logger()

        if self.state == RaceState.INIT:
            log.info('📍 [INIT] 设置 AMCL 初始位姿...')
            self.nav.set_initial_pose(*self.P_POINT)
            # 给 AMCL 一点时间收敛粒子
            rclpy.spin_once(self.nav, timeout_sec=2.0)
            self.state = RaceState.GO_TO_QR

        elif self.state == RaceState.GO_TO_QR:
            log.info(f'📍 [GO_TO_QR] P → 二维码点: {self.QR_POINT}')
            if self.nav.go_to(*self.QR_POINT, timeout_sec=self.WAYPOINT_TIMEOUT):
                self.state = RaceState.DECODE_QR
            else:
                log.error('❌ 到二维码任务点失败！')
                self.state = RaceState.FAILED

        elif self.state == RaceState.DECODE_QR:
            log.info('📍 [DECODE_QR] 解码二维码...')

            # ======================== Gen1: 写死 ========================
            qr_value = 1  # 奇数 → 顺时针
            log.info(f'  🔧 Gen1 写死二维码值: {qr_value} (奇→顺, 偶→逆)')
            # ============================================================
            # Gen2 替换为:
            #   qr_value = self.camera.decode_qr()
            #   if qr_value is None: 重试或失败
            # ============================================================

            self.wp.set_direction_from_qr(qr_value)
            log.info(f'  ➡️  椭圆走行方向: {self.wp.direction}')

            self.state = RaceState.DO_ELLIPSE

        elif self.state == RaceState.DO_ELLIPSE:
            log.info(f'📍 [DO_ELLIPSE] 走椭圆 — {self.wp.direction}')

            for lap in range(self.wp.num_laps):
                log.info(f'━━━ 第 {lap + 1}/{self.wp.num_laps} 圈 ━━━')
                waypoints = self.wp.get_waypoints_for_lap(lap)

                for i, (wx, wy, wyaw) in enumerate(waypoints):
                    log.info(f'  → Waypoint {i}/{len(waypoints)-1}: ({wx:.2f}, {wy:.2f})')

                    # ==================== Gen2 扩展点 ====================
                    # 如果当前 waypoint 靠近人形立牌位置:
                    #   if i in self.wp.get_standee_waypoints():
                    #       self._handle_standee()  # 识别+大模型+显示屏
                    # ====================================================

                    if not self.nav.go_to(wx, wy, wyaw,
                                          timeout_sec=self.WAYPOINT_TIMEOUT):
                        log.error(f'❌ Waypoint {i} 失败！')
                        self.state = RaceState.FAILED
                        return

                self.lap_count = lap + 1

            log.info(f'✅ 椭圆走完 ({self.wp.num_laps} 圈), 返回 P 点')
            self.state = RaceState.RETURN_P

        elif self.state == RaceState.RETURN_P:
            log.info(f'📍 [RETURN_P] 返回出发点 P: {self.P_POINT}')
            if self.nav.go_to(*self.P_POINT, timeout_sec=self.WAYPOINT_TIMEOUT):
                self.state = RaceState.FINISHED
            else:
                log.error('❌ 返回 P 点失败！')
                self.state = RaceState.FAILED

        elif self.state == RaceState.FINISHED:
            pass  # run() 循环会退出

        elif self.state == RaceState.FAILED:
            pass  # run() 循环会退出

    # ==================== Gen2 函数 (占位) ====================

    # def _handle_standee(self):
    #     """
    #     人形立牌处理流程 (Gen2):
    #     1. 深度相机拍摄
    #     2. 大模型识别图文内容
    #     3. 把内容显示到显示屏
    #     """
    #     log = self.nav.get_logger()
    #     log.info('🪧 检测到人形立牌，启动识别流程...')
    #     # image = self.camera.capture()
    #     # text = self.llm.recognize(image)
    #     # self.display.show(text)
    #     # log.info(f'立牌内容: {text}')


# ==================== 入口 ====================

def main(args=None):
    rclpy.init(args=args)

    sm = RaceStateMachine()

    try:
        sm.run()
    except KeyboardInterrupt:
        sm.nav.get_logger().info('⚠️  用户中断')
    except Exception as e:
        sm.nav.get_logger().fatal(f'💥 未捕获异常: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # 不用 rclpy.shutdown() 因为 navigator 节点可能还被 spin_until_future_complete 引用
        rclpy.shutdown()


if __name__ == '__main__':
    main()
