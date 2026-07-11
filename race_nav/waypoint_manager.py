#!/usr/bin/env python3
"""
waypoint_manager.py — 椭圆赛道航点管理器。

======================== 使用说明 ========================
Gen1 (当前):  从硬编码列表加载 8 个椭圆航点
Gen2 扩展点:  从 YAML 加载 + 根据 QR 解码结果决定顺时针/逆时针
======================== 使用说明 ========================

=== 如何获取准确的 waypoint 坐标 ===
方法: 在 RViz2 中用 "Publish Point" 工具 (工具栏按钮) 沿着椭圆赛道点击,
同时在终端运行:
    ros2 topic echo /clicked_point
记录下每个点的 (x, y), yaw 用 atan2(Δy, Δx) 算出来或直接填 0 也行
──────────────────────────────────────────────────

=== 椭圆示意图 (8 点近似) ===

                ⑦  ╲     ╱  ①
              ╱              ╲
           ⑥                    ②         顺时针走: ①→②→③→④→⑤→⑥→⑦→⑧
           │         中心         │
           ⑤                    ③         逆时针走: ①→⑧→⑦→⑥→⑤→④→③→②
              ╲              ╱
                ④       ③
                    ②
"""

from math import pi


class WaypointManager:
    """
    椭圆柱点管理器。
    Gen1: 硬编码坐标
    Gen2: 从 YAML 加载 + 根据二维码值决定方向
    """

    # ================================================================
    # ⚠️  在这里改你的椭圆 waypoint 坐标！
    # 格式: (x, y, yaw)  — x, y 单位米, yaw 单位弧度
    # 获取方法: RViz2 → Publish Point → 在椭圆上点 8~12 个点 → 记坐标
    # ================================================================
    ELLIPSE_WAYPOINTS = [
        # idx  (x,   y,    yaw)      描述
        (2.0,  0.0,  0.0),          # 0: 椭圆右端
        (1.4,  0.8,  pi * 0.125),   # 1: 右上
        (0.0,  1.5,  pi * 0.25),    # 2: 正上
        (-1.4, 0.8,  pi * 0.375),   # 3: 左上
        (-2.0, 0.0,  pi * 0.5),     # 4: 椭圆左端
        (-1.4, -0.8, pi * 0.625),   # 5: 左下
        (0.0,  -1.5, pi * 0.75),    # 6: 正下
        (1.4,  -0.8, pi * 0.875),   # 7: 右下
    ]

    # 椭圆圈数
    DEFAULT_LAPS = 2

    def __init__(self, direction: str = 'clockwise'):
        """
        Args:
            direction: 'clockwise' (顺时针) 或 'counterclockwise' (逆时针)
        """
        self.direction = direction
        self.num_laps = self.DEFAULT_LAPS

    # ==================== 方向控制 ====================

    def set_direction(self, direction: str):
        """
        设置行走方向。

        Gen1: 写死调用 set_direction('clockwise')
        Gen2: 二维码解码后调用 set_direction_from_qr(qr_value)
        """
        if direction not in ('clockwise', 'counterclockwise'):
            raise ValueError(f"direction 必须是 'clockwise' 或 'counterclockwise', 收到: {direction}")
        self.direction = direction

    def set_direction_from_qr(self, qr_value: int):
        """
        根据二维码数值设置方向: 奇顺偶逆

        Gen1: 不调用这个, 方向写死
        Gen2: qr_value = decode_qr(...) → self.set_direction_from_qr(qr_value)
        """
        if qr_value % 2 == 1:
            self.direction = 'clockwise'
        else:
            self.direction = 'counterclockwise'

    # ==================== 航线获取 ====================

    def get_waypoints_for_lap(self, lap: int = 0) -> list:
        """
        返回第 lap 圈应该走的 waypoint 列表。

        Args:
            lap: 第几圈 (从 0 开始)

        Returns:
            [(x, y, yaw), ...] 按 direction 排好序的航点列表

        顺时针: [0, 1, 2, 3, 4, 5, 6, 7]
        逆时针: [0, 7, 6, 5, 4, 3, 2, 1]
        """
        if self.direction == 'clockwise':
            return self.ELLIPSE_WAYPOINTS
        else:
            # 逆时针: 起点 0 不动, 然后倒序走
            return [self.ELLIPSE_WAYPOINTS[0]] + \
                   list(reversed(self.ELLIPSE_WAYPOINTS[1:]))

    def get_total_waypoints(self) -> int:
        """返回总共要走的航点数 (圈数 × 每圈点数)"""
        return len(self.ELLIPSE_WAYPOINTS) * self.num_laps

    # ==================== 人形立牌位置 (Gen2) ====================

    # Gen2: 预设人形立牌在椭圆两侧的大致位置
    # 车在经过这些 waypoint 附近时会触发相机识别
    STANDEE_POSITIONS = [
        # (waypoint_index, side) — 在哪个 waypoint 附近、椭圆哪一侧
        (2, 'outer'),   # 上方 waypoint 附近, 椭圆外侧
        (6, 'outer'),   # 下方 waypoint 附近, 椭圆外侧
    ]

    def get_standee_waypoints(self) -> list:
        """
        返回可能有人形立牌的 waypoint index 列表 (Gen2 用)。
        当车靠近这些航点时, 触发相机识别 + 大模型 + 显示屏。
        """
        return [wp_idx for wp_idx, _ in self.STANDEE_POSITIONS]
