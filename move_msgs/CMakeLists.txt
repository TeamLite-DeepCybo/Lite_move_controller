cmake_minimum_required(VERSION 3.16)
project(move_msgs)

# ---------------------------------------------------------------------------
# move_msgs —— 运控层（lite_move_controller）与**上游数据源**之间的接口包
#
# 与 robstride_msgs 的**刻意解耦**（这是本包存在的唯一理由）：
#
#   move_msgs       运控层 ↔ 上游：终端位姿目标、状态反馈、可达性查询
#   robstride_msgs  运控层 ↔ 桥 ↔ 控制器 ↔ 驱动：电机命令与电机状态
#
#   两者**互不依赖**。运控层同时依赖两者，并在内部做单向转换：
#       move_msgs/MoveTarget  ──▶（滤波+IK+控制律）──▶  robstride_msgs/MotionCommand
#
#   为什么不合并进 robstride_msgs：
#     · 终端位姿与电机协议毫无关系，混在一起会让"改 IK 参数"牵动"电机帧定义"；
#     · robstride_msgs 是**机器人基础信息**的一部分（驱动/控制器/桥都依赖它），
#       本项目的铁律是**不修改**它 —— 新增运控层只能在新包里加契约。
# ---------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ament_package() 依赖 catkin_pkg；本机只有系统 python3 装了该模块，
# 而 CMake 默认可能选中 ~/.local 下的解释器，故显式钉住（与 robstride_msgs 一致）。
if(EXISTS "/usr/bin/python3")
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter for ament" FORCE)
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # --- 上游 → 运控层：终端目标（流式）---
  "msg/ControlBatch.msg"
  "srv/ClaimControlStream.srv"
  "msg/MoveTarget.msg"
  "msg/MoveTargetArray.msg"
  # --- 运控层 → 上游：状态与残差反馈 ---
  "msg/MoveStatus.msg"
  "msg/EeResidual.msg"
  "msg/InteractionEvent.msg"
  # --- 查询 ---
  "srv/GetMoveStatus.srv"
  "srv/QueryIk.srv"
  "srv/QueryFk.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
