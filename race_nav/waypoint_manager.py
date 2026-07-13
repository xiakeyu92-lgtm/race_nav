#!/usr/bin/env python3
"""
waypoint_manager.py — 赛道航点管理器。

======================== 赛道结构 ========================

    Area C (椭圆环形)
    圈12(0.75,3.7) ← 圈11(0.75,4.85) ← 圈10(2,4.85) ← 圈9(3,4.85) ← 圈8(4.25,4.85)
      │                                                                  ↑
      ↓                                                                  │
    圈6(3,3.7) ──────────→ 圈7(4.25,3.7) ──────────────────────────────┘

    Area B (黄色通道 — 两侧有墙，不能压！)
    圈5(2.5,1.22) → 圈4(2.5,1.97) → 圈3(2.5,2.5)

    Area A (蓝白区域)
    P点(0.3,0.3) ······ 二维码(4.75,2.3)

======================== 行进路线 ========================
顺时针: P → QR → 圈3→圈4→圈5 → 圈6→圈7→圈8→圈9→圈10→圈11→圈12 → 回到P
逆时针: P → QR → 圈3→圈4→圈5 → 圈12→圈11→圈10→圈9→圈8→圈7→圈6 → 回到P

⚠️ 黄色通道两侧有墙，waypoint 必须标在通道中心线上！
   Nav2 的 A* 会自动沿着通道规划路径（costmap 把墙标为障碍物）
======================== 行进路线 ========================
"""


class WaypointManager:
    """
    赛道航点管理器。
    Gen1: 硬编码坐标，写死方向
    Gen2: 从 YAML 加载 + 根据二维码值决定方向
    """

    # ================================================================
    # 黄色通道入口 (Area B) — 从 Area A 进入 Area C 的必经之路
    # 两侧有墙，waypoint 标在通道中心线
    # ================================================================
    CHANNEL_WAYPOINTS = [
        # idx  (x,     y,    yaw)   描述
        (2.5,  2.5,   0.0),        # 圈3: 通道上端入口
        (2.5,  1.97,  0.0),        # 圈4: 通道中段
        (2.5,  1.22,  0.0),        # 圈5: 通道末端，接近椭圆
    ]

    # ================================================================
    # 椭圆赛道 (Area C) — 黄色环形消毒巡航区
    # 顺时针: 圈6→圈7→圈8→圈9→圈10→圈11→圈12
    # ================================================================
    ELLIPSE_WAYPOINTS = [
        # idx  (x,     y,     yaw)   描述
        (3.0,   3.7,   0.0),        # 圈6: 椭圆右下
        (4.25,  3.7,   0.0),        # 圈7: 椭圆右中
        (4.25,  4.85,  0.0),        # 圈8: 椭圆右上
        (3.0,   4.85,  0.0),        # 圈9: 椭圆上中
        (2.0,   4.85,  0.0),        # 圈10: 椭圆左上
        (0.75,  4.85,  0.0),        # 圈11: 椭圆左中
        (0.75,  3.7,   0.0),        # 圈12: 椭圆左下
    ]

    # 椭圆圈数
    DEFAULT_LAPS = 1  # 比赛要求沿黄色通道行驶一周

    def __init__(self, direction: str = 'clockwise'):
        """
        Args:
            direction: 'clockwise' (顺时针) 或 'counterclockwise' (逆时针)
        """
        self.direction = direction
        self.num_laps = self.DEFAULT_LAPS

    # ==================== 方向控制 ====================

    def set_direction(self, direction: str):
        """设置走行方向。"""
        if direction not in ('clockwise', 'counterclockwise'):
            raise ValueError(
                f"direction 必须是 'clockwise' 或 'counterclockwise', 收到: {direction}")
        self.direction = direction

    def set_direction_from_qr(self, qr_value: int):
        """
        根据二维码数值设置方向: 奇顺偶逆。
        Gen1: 不调用这个，方向写死。
        Gen2: qr_value = decode_qr(...) → self.set_direction_from_qr(qr_value)
        """
        if qr_value % 2 == 1:
            self.direction = 'clockwise'
        else:
            self.direction = 'counterclockwise'

    # ==================== 航点获取 ====================

    def get_channel_waypoints(self) -> list:
        """返回黄色通道段的航点列表（单向进入）。"""
        return self.CHANNEL_WAYPOINTS

    def get_ellipse_waypoints(self) -> list:
        """
        返回椭圆段的航点列表，按方向排序：
        顺时针: [圈6, 圈7, 圈8, 圈9, 圈10, 圈11, 圈12]
        逆时针: [圈12, 圈11, 圈10, 圈9, 圈8, 圈7, 圈6]
        """
        if self.direction == 'clockwise':
            return self.ELLIPSE_WAYPOINTS
        else:
            # 逆时针: 从圈12开始往回走
            return list(reversed(self.ELLIPSE_WAYPOINTS))

    def get_total_waypoints(self) -> int:
        """返回总共要走的航点数"""
        channel_count = len(self.CHANNEL_WAYPOINTS)
        ellipse_count = len(self.ELLIPSE_WAYPOINTS) * self.num_laps
        return channel_count + ellipse_count

    # ==================== 人形立牌位置 (Gen2) ====================

    # Gen2: 预设人形立牌在椭圆两侧的大致位置
    # 车在经过这些 waypoint 附近时触发相机识别
    STANDEE_POSITIONS = [
        (3, 'outer'),   # 圈9 上方 waypoint 附近
        (6, 'outer'),   # 圈12 左侧 waypoint 附近
    ]

    def get_standee_indices(self) -> list:
        """返回可能有人形立牌的椭圆 waypoint index 列表 (Gen2 用)。"""
        return [wp_idx for wp_idx, _ in self.STANDEE_POSITIONS]