#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# filepath: src/scout_nav/scripts/dynamic_tf_broadcaster.py
# ============================================================================
# 动态双 TF 广播器节点——双阶全归零在线标定的核心引擎
#
# 本节点在 50Hz 循环中同时广播两条 TF 变换：
#
#   TF-1: base_footprint → base_link
#         仅 Z 轴平移，取 base_z_offset 滑块值
#         物理意义：底盘几何中心距真实地面的高度
#
#   TF-2: base_link → velodyne
#         完整 6 自由度（x, y, z, roll, pitch, yaw）
#         物理意义：雷达相对于车体中心的安装位姿
#
# 当 rqt_reconfigure 的滑块被拖动时，回调函数立即更新内部参数值，
# 下一个 50Hz 周期就会广播更新后的 TF，实现"拖滑块→看效果"的零延迟体验
#
# 使用场景：仅在标定阶段（calibrate_extrinsics.launch）中启动！
#           标定完成后，base_footprint 被废弃，此节点不再需要。
# ============================================================================

import rospy
import tf2_ros
import geometry_msgs.msg

# tf_conversions 提供 euler ↔ quaternion 互转函数
# quaternion_from_euler(roll, pitch, yaw) → (qx, qy, qz, qw)
import tf_conversions

# 导入 dynamic_reconfigure 服务端
from dynamic_reconfigure.server import Server

# ★ 导入 catkin_make 自动生成的配置类（基于 cfg/Extrinsics.cfg）
# 该类包含 7 个参数：base_z_offset, x, y, z, roll, pitch, yaw
from scout_nav.cfg import ExtrinsicsConfig


class DynamicTFBroadcasterNode:
    """
    双 TF 动态广播器节点

    职责：
    1. 托管 Extrinsics.cfg 定义的 7 个动态参数
    2. 以 50Hz 同时广播 base_footprint→base_link 和 base_link→velodyne 两条 TF
    3. 当 rqt_reconfigure 滑块被拖动时，回调函数实时更新参数
    """

    def __init__(self):
        # ============ 初始化 ROS 节点 ============
        # anonymous=False: 节点名固定为 dynamic_tf_broadcaster
        # rqt_reconfigure 通过固定节点名来显示参数面板
        rospy.init_node("dynamic_tf_broadcaster", anonymous=False)

        # ============ 初始化 TF2 广播器 ============
        # TransformBroadcaster 发布动态 TF 到 /tf 话题
        # 动态 TF 的优先级高于同名的静态 TF
        self.br = tf2_ros.TransformBroadcaster()

        # ============ 第一阶参数：底盘离地高度（初始 0） ============
        self.base_z_offset = 0.0

        # ============ 第二阶参数：雷达 6 自由度外参（初始全 0） ============
        self.lidar_x = 0.0      # 前后偏移（前方为正），单位：米
        self.lidar_y = 0.0      # 左右偏移（左方为正），单位：米
        self.lidar_z = 0.0      # 上下偏移（上方为正），单位：米
        self.lidar_roll = 0.0   # 绕 X 轴滚转，单位：弧度
        self.lidar_pitch = 0.0  # 绕 Y 轴俯仰，单位：弧度
        self.lidar_yaw = 0.0    # 绕 Z 轴偏航，单位：弧度

        # ============ 启动 dynamic_reconfigure 服务端 ============
        # Server(ConfigType, callback)
        # 启动时会立即调用一次 callback，传入 cfg 中的 default 值
        self.server = Server(ExtrinsicsConfig, self.reconfigure_callback)

        # ============ 设置广播频率 ============
        # 50Hz: 每 20ms 广播一次 TF，足够 RViz 实时刷新
        self.rate = rospy.Rate(50)

        rospy.loginfo("=" * 60)
        rospy.loginfo("双阶动态 TF 广播器已启动！")
        rospy.loginfo("广播中：base_footprint -> base_link（底盘高度）")
        rospy.loginfo("广播中：base_link -> velodyne（雷达外参）")
        rospy.loginfo("-" * 60)
        rospy.loginfo("请在终端执行：rosrun rqt_reconfigure rqt_reconfigure")
        rospy.loginfo("第一阶：拖动 base_z_offset 抬升车体直到车轮触地")
        rospy.loginfo("第二阶：拖动 x/y/z/roll/pitch/yaw 对齐雷达点云")
        rospy.loginfo("=" * 60)

    def reconfigure_callback(self, config, level):
        """
        dynamic_reconfigure 参数变更回调函数

        当 rqt_reconfigure GUI 中任何一个滑块被拖动时，此函数被即时调用。
        每次调用都会收到全部 7 个参数的最新值。

        参数：
            config: 包含所有参数最新值的字典对象
                    config.base_z_offset: 第一阶底盘高度
                    config.x/y/z/roll/pitch/yaw: 第二阶雷达外参
            level:  变更位掩码（本工程中所有参数 level=0，不使用）

        返回：
            config: 必须原样返回（dynamic_reconfigure 框架硬性要求）
        """
        # ===== 更新第一阶参数 =====
        self.base_z_offset = config.base_z_offset

        # ===== 更新第二阶参数 =====
        self.lidar_x = config.x
        self.lidar_y = config.y
        self.lidar_z = config.z
        self.lidar_roll = config.roll
        self.lidar_pitch = config.pitch
        self.lidar_yaw = config.yaw

        # 终端打印当前全部参数值，便于实时监控
        rospy.loginfo(
            "[第一阶] base_z_offset=%.4f | "
            "[第二阶] X=%.4f Y=%.4f Z=%.4f Roll=%.4f Pitch=%.4f Yaw=%.4f",
            self.base_z_offset,
            self.lidar_x, self.lidar_y, self.lidar_z,
            self.lidar_roll, self.lidar_pitch, self.lidar_yaw
        )

        # 必须返回 config
        return config

    def _build_transform(self, parent, child, x, y, z, roll, pitch, yaw):
        """
        构造一个 TransformStamped 消息

        参数：
            parent: 父坐标系名称
            child:  子坐标系名称
            x, y, z: 平移分量（米）
            roll, pitch, yaw: 旋转分量（弧度），使用 ZYX 欧拉角顺序

        返回：
            geometry_msgs.msg.TransformStamped 实例
        """
        t = geometry_msgs.msg.TransformStamped()

        # 时间戳：使用当前 ROS 时间
        # 实车模式下为系统时钟，rosbag 模式下为 /clock 时间
        t.header.stamp = rospy.Time.now()

        # 坐标系 ID
        t.header.frame_id = parent
        t.child_frame_id = child

        # 平移分量
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # 旋转分量：欧拉角 → 四元数
        # quaternion_from_euler(roll, pitch, yaw) 返回 (qx, qy, qz, qw)
        q = tf_conversions.transformations.quaternion_from_euler(
            roll, pitch, yaw
        )
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

    def broadcast_all(self):
        """
        同时广播两条 TF 变换（每个 50Hz 周期调用一次）

        TF-1: base_footprint → base_link
              仅 Z 轴平移 = base_z_offset
              X/Y/Roll/Pitch/Yaw 全为 0（底盘不侧移不倾斜）

        TF-2: base_link → velodyne
              完整 6 自由度 = lidar_x/y/z/roll/pitch/yaw
        """
        # ===== TF-1: 底盘高度（base_footprint → base_link） =====
        # base_footprint 固定在 Z=0 地面
        # base_link 在其正上方 base_z_offset 处
        # 只有 Z 平移，其他 5 个自由度全为 0
        tf_chassis = self._build_transform(
            parent="base_footprint",
            child="base_link",
            x=0.0,
            y=0.0,
            z=self.base_z_offset,  # ★ 第一阶滑块控制的唯一变量
            roll=0.0,
            pitch=0.0,
            yaw=0.0
        )

        # ===== TF-2: 雷达外参（base_link → velodyne） =====
        # 完整 6 自由度由第二���的 6 个滑块控制
        tf_lidar = self._build_transform(
            parent="base_link",
            child="velodyne",
            x=self.lidar_x,
            y=self.lidar_y,
            z=self.lidar_z,
            roll=self.lidar_roll,
            pitch=self.lidar_pitch,
            yaw=self.lidar_yaw
        )

        # 使用一次 sendTransform 调用同时发送两条 TF
        # 这比分两次调用更高效，且保证时间戳一致
        self.br.sendTransform([tf_chassis, tf_lidar])

    def run(self):
        """
        节点主循环：以 50Hz 持续广播双 TF 直到节点关闭
        """
        while not rospy.is_shutdown():
            self.broadcast_all()
            self.rate.sleep()


# ============ 程序入口 ============
if __name__ == "__main__":
    try:
        node = DynamicTFBroadcasterNode()
        node.run()
    except rospy.ROSInterruptException:
        # 捕获 Ctrl+C 中断，优雅退出
        pass