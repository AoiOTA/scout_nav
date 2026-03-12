#!/bin/bash
# filepath: src/scout_nav/scripts/clean_ros_env.sh
# ============================================================================
# 僵尸进程与时钟重置一键清场脚本 (Nuclear Cleanup SOP)
#
# 痛点解决：
# 在实车跑图 (use_sim_time=false) 和 Bag 回放 (use_sim_time=true) 切换时，
# 后台常会残留 roscore 参数缓存或死锁的底盘 CAN 节点，导致 TF 时间戳撕裂
# (如报 TF_REPEATED_DATA 或 Jump back in time of XXXX s)。
#
# 作用：暴力斩杀所有 ROS 守护进程，释放端口，彻底重置参数服务器！
# ============================================================================

echo "====================================================="
echo "💣 [ROS Cleanup] 准备执行核弹级清理程序..."
echo "====================================================="

# 1. 尝试优雅注销节点
echo "➡️  正在向 ROS Master 发送节点注销请求..."
rosnode kill -a 2>/dev/null

# 2. 暴力斩杀三大核心进程
echo "➡️  正在物理击杀 roscore / rosmaster / rosout..."
killall -9 roscore rosmaster rosout 2>/dev/null

# 3. 斩杀常见高负荷节点
echo "➡️  正在清理 Rviz, Gmapping, AMCL, MoveBase..."
killall -9 rviz slam_gmapping amcl move_base pointcloud_to_laserscan_node 2>/dev/null

# 4. 重点狙击：底盘驱动与 TF 广播器 (极易成为僵尸进程)
echo "➡️  正在清除底层硬件驻留进程 (scout_base_node)..."
pkill -9 -f scout_base_node
pkill -9 -f dynamic_tf_broadcaster

# 5. 清理 ROS 日志缓存 (防止硬盘占满)
echo "➡️  正在清理过期 ROS 日志文件..."
rosclean purge -y > /dev/null 2>&1

echo "====================================================="
echo "✅ 清理完成！你的系统现在绝对干净了。"
echo "⚠️  注意：如果接下来要跑 Rosbag，请按以下顺序执行："
echo "   1. roscore &"
echo "   2. rosparam set /use_sim_time true"
echo "   3. roslaunch scout_nav scout_xxx_offline.launch"
echo "====================================================="