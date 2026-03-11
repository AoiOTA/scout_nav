# filepath: src/scout_nav/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(scout_nav)

## 查找 catkin 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  xacro
  robot_state_publisher
  joint_state_publisher       # ★ 发布虚拟关节状态，让 RViz 中车轮能渲染
  dynamic_reconfigure         # ★ 运行时动态参数重配置框架
  pointcloud_to_laserscan     # ★ 3D 点云降维为 2D LaserScan
)

## ★ 生成动态重配置参数（读取 cfg/ 下的 .cfg 文件）
## catkin_make 时自动生成：
##   devel/include/scout_nav/ExtrinsicsConfig.h
##   devel/lib/python3/dist-packages/scout_nav/cfg/ExtrinsicsConfig.py
generate_dynamic_reconfigure_options(
  cfg/Extrinsics.cfg
)

## catkin 包声明
catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs nav_msgs
    tf2 tf2_ros xacro
    robot_state_publisher joint_state_publisher
    dynamic_reconfigure pointcloud_to_laserscan
)

## 头文件包含路径
include_directories(${catkin_INCLUDE_DIRS})

## 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/dynamic_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装资源目录
install(DIRECTORY launch param rviz maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)