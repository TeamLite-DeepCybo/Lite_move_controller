cmake_minimum_required(VERSION 3.16)
project(lite_move_controller)

# ---------------------------------------------------------------------------
# lite_move_controller —— 运控层（控制律 + 与桥对接）
#
# 分层（**依赖方向单向**）：
#
#   上游数据源 ──move_msgs/MoveTarget──▶ 本包 ──robstride_msgs/MotionCommand──▶ lite_bridge
#                                          │
#                                          ├── lite_ik_node（运动学库与模型资产）
#                                          └── lite_dyid 的 params.json
#
# 纯 C++ 核心（无 ROS 依赖）与 ROS 外壳分开放：
#   include/lite_move_controller/{butterworth,resampler,admittance,impedance,
#                                 limit_guard,node_state}.hpp  ← 零 ROS，可裸测
#   src/*_node.cpp                                            ← ROS 外壳
#   理由：守卫/控制律是安全关键逻辑，必须能脱离 ROS 穷举单测
#        （与 lite_bridge 的 source_arbiter、lite_controller 的 activation_ramp 同一理由）。
# ---------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra)

# ament 需要 catkin_pkg；显式钉住 /usr/bin/python3（与 robstride_msgs 一致）。
if(EXISTS "/usr/bin/python3")
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter for ament" FORCE)
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "Python interpreter" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# pinocchio：**与 lite_dyid 辨识侧同库**。动力学的正确性靠"用同一个库、
# 同一句 API"保证，而不是靠我重新实现刚体物理（那一版与辨识不等价）。
find_package(pinocchio REQUIRED)

# ⚠️ yaml-cpp：**必须钉住系统的 0.8**，否则会链到 conda 的 0.9。
#    现象（实测）：编译链接都过，运行时却报
#        error while loading shared libraries: libyaml-cpp.so.0.9: cannot open shared object file
#    根因：`find_package(yaml-cpp)` 走 PATH，而 PATH 里有 anaconda3，
#          于是拿到 ~/anaconda3/lib/libyaml-cpp.so.0.9（系统只有 0.8，
#          conda 的 .so 不在运行时搜索路径里）。
#    这正是本项目文档反复警告的 conda 污染（lite_controller 的 README §1.5）。
#    这里显式排除 conda 前缀，并优先用系统的 cmake 配置。
set(CMAKE_IGNORE_PATH
    "${CMAKE_IGNORE_PATH}"
    "$ENV{HOME}/anaconda3;$ENV{HOME}/anaconda3/lib;$ENV{HOME}/anaconda3/lib/cmake"
    "$ENV{HOME}/miniconda3;$ENV{HOME}/miniconda3/lib;$ENV{HOME}/miniconda3/lib/cmake")
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PATH
    "$ENV{CONDA_PREFIX}" "$ENV{CONDA_PREFIX}/lib" "$ENV{CONDA_PREFIX}/lib/cmake")
endif()
# 显式指定系统 yaml-cpp 的 cmake 目录（存在就用它）
find_package(yaml-cpp QUIET CONFIG
             PATHS /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp
             NO_DEFAULT_PATH)
if(NOT yaml-cpp_FOUND)
  find_package(yaml-cpp REQUIRED)
endif()
message(STATUS "yaml-cpp: ${yaml-cpp_VERSION} (${yaml-cpp_DIR})")

# ⚠️⚠️ `Boost_USE_DEBUG_RUNTIME` 必须显式关掉，否则 moveit_core 的依赖链会
#    在**配置期**炸掉（实测，且报错内容与真实原因完全不搭边）：
#
#      CMake Error at .../BoostConfig.cmake:141 (find_package):
#        Found package configuration file: .../boost_random-1.83.0/...
#        but it set boost_random_FOUND to FALSE ... No suitable build variant
#        has been found:
#          * libboost_random.so.1.83.0 (release runtime, Boost_USE_DEBUG_RUNTIME=TRUE)
#          * libboost_random.a (static, Boost_USE_STATIC_LIBS=OFF)
#
#    根因链条（三段，缺一不可）：
#      ① `pinocchio` 的 cmake 配置走的是 **module 模式** `FindBoost.cmake`；
#      ② `FindBoost.cmake`（本机 cmake 3.28）在变量未定义时会
#         `set(Boost_USE_DEBUG_RUNTIME TRUE)` —— 这是历史上**只对 Windows
#         有意义**的开关（挑 debug 运行库），在 Linux 上纯属误导；
#      ③ 这个变量**泄漏到同一目录作用域**，随后 moveit_core →
#         geometric_shapes → random_numbers 用 **CONFIG 模式**找
#         `boost_random`，而 Ubuntu 只装了 release 版 Boost 库 ⇒ 按 TRUE 去要
#         debug 运行库必然找不到 ⇒ boost_random_FOUND=FALSE ⇒ 整包配置失败。
#
#    ⚠️ 所以修法必须放在 `find_package(pinocchio)` **之后**、`find_package(moveit_core)`
#       之前：放在最前面会被 pinocchio 的 FindBoost 重新置回 TRUE（实测）。
#       在本机（Linux，只有 release 版 Boost）将它固定为 OFF 是**正确**语义，
#       不是绕过检查。
set(Boost_USE_DEBUG_RUNTIME OFF)
find_package(lite_ik_node REQUIRED)
find_package(urdf REQUIRED)

# 运控层的消息契约（**独立于 robstride_msgs**）
find_package(move_msgs REQUIRED)
# 向下：仍然严格遵循电机侧的契约
find_package(robstride_msgs REQUIRED)

# --- 纯 C++ 核心 -----------------------------------------------------------
#
# ⚠️ 故意做成 INTERFACE 库（只有头文件、零依赖）：单测可以
#    `g++ -std=c++17 -I include test/test_core.cpp` 裸跑，
#    不需要 colcon、不需要 ROS、0.1 秒出结果。
add_library(lite_move_controller_core INTERFACE)
target_include_directories(lite_move_controller_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(lite_move_controller_core INTERFACE cxx_std_17)
target_link_libraries(lite_move_controller_core INTERFACE
  lite_ik_node::lite_ik_node_types)

# --- 动力学前馈 ------------------------------------------------------------
add_library(lite_move_controller_dynamics STATIC
  src/dynamics_pin.cpp
  src/gravity_yaml.cpp)
target_include_directories(lite_move_controller_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ⚠️ 链接名踩过坑：Jazzy 的 orocos_kdl_vendor **不导出** `orocos_kdl_vendor::orocos-kdl`
#    ALIAS，它只在 extras.cmake 里 `find_package(orocos_kdl)` 后造出名为
#    **`orocos-kdl`** 的 IMPORTED target。写错会得到
#    "An ALIAS target is missing" 这种与真实原因无关的报错。
# ⚠️ 必须用 **plain 形式**：`ament_target_dependencies` 内部是 plain
#    `target_link_libraries(<t> <libs...>)`，两者混用会报
#    "all uses must be either all-keyword or all-plain"。
target_link_libraries(lite_move_controller_dynamics
  lite_move_controller_core)
ament_target_dependencies(lite_move_controller_dynamics Eigen3 urdf)
ament_target_dependencies(lite_move_controller_dynamics rclcpp)
# pinocchio 用 target link（其 cmake 提供 IMPORTED target）
target_link_libraries(lite_move_controller_dynamics pinocchio::pinocchio pinocchio::pinocchio_parsers)
# yaml-cpp 的包名与 target 名不同，需要显式链接导出的 target。
target_link_libraries(lite_move_controller_dynamics yaml-cpp::yaml-cpp)

# --- 桥客户端 + ROS 节点（需要 rclcpp / robstride_msgs / move_msgs）----------
add_library(lite_move_controller_bridge STATIC src/bridge_client.cpp)
target_include_directories(lite_move_controller_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lite_move_controller_bridge lite_move_controller_core)
ament_target_dependencies(lite_move_controller_bridge
  rclcpp robstride_msgs sensor_msgs std_msgs)

# 主节点：接线层（所有决策都在纯核心与 IK 后端里）
add_executable(move_controller_node src/move_controller_node.cpp)
target_link_libraries(move_controller_node
  lite_move_controller_bridge lite_move_controller_dynamics
  lite_ik_node::lite_ik_node_kinematics)
ament_target_dependencies(move_controller_node
  rclcpp std_msgs geometry_msgs sensor_msgs std_srvs move_msgs robstride_msgs)

# 测试数据源：左手圆 / 右手方形（S6）。**只在测试时起**，不属于运控链路的必需件。
add_executable(move_shape_source src/shape_source_node.cpp)
target_link_libraries(move_shape_source lite_move_controller_core)
ament_target_dependencies(move_shape_source rclcpp std_msgs geometry_msgs move_msgs)

# --- 纯 C++ 单测（裸跑，不注册到 colcon test）--------------------------------
#
# 这三个可执行文件**不依赖 ROS**，因此可以在任何机器上跑：
#     ./build/lite_move_controller/test_core
#     ./build/lite_move_controller/test_limits_from_yaml
# 契约测试（test_state_contract）需要 move_msgs 生成头，故走 normal target。
add_executable(test_core test/test_core.cpp)
target_link_libraries(test_core PRIVATE lite_move_controller_core)

# 输入滤波管线 + 测试数据源形状（纯逻辑，零 ROS）
add_executable(test_filter_shape test/test_filter_shape.cpp)
target_link_libraries(test_filter_shape PRIVATE lite_move_controller_core)

# 夹爪填充逻辑（纯函数，零 ROS）：14/16 关节模式、缺关节、帧长度不符、下标越界
add_executable(test_gripper test/test_gripper.cpp)
target_link_libraries(test_gripper PRIVATE lite_move_controller_core)

# 动力学 + 雅可比集成测试（需要真实 URDF 与 schema2 的 params.json）
add_executable(test_dynamics test/test_dynamics.cpp)
target_link_libraries(test_dynamics lite_move_controller_dynamics lite_ik_node::lite_ik_node_kinematics)

# 对拍工具：输出 C++ 侧的 τ_g / τ_friction / τ_total（供 cmp_pin_vs_python.py 用）
add_executable(dump_pin test/tools/dump_pin.cpp)
target_link_libraries(dump_pin lite_move_controller_dynamics lite_ik_node::lite_ik_node_kinematics)

# 契约测试：把"内部枚举 == 对外 msg 常量"变成**编译期**约束
if(TARGET move_msgs::move_msgs__rosidl_generator_cpp)
  add_executable(test_state_contract test/test_state_contract.cpp)
  # ⚠️ 必须**统一用 plain 形式**：`ament_target_dependencies` 内部用的是
  #    plain `target_link_libraries(<t> <libs...>)`，若这里先用了 keyword 形式
  #    （PRIVATE），CMake 会报 "all uses must be either all-keyword or all-plain"。
  target_link_libraries(test_state_contract lite_move_controller_core)
  ament_target_dependencies(test_state_contract move_msgs)
endif()

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS move_controller_node move_shape_source
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS lite_move_controller_dynamics lite_move_controller_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_export_include_directories(include)
ament_export_libraries(lite_move_controller_dynamics lite_move_controller_bridge)
ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs std_srvs move_msgs robstride_msgs
                        lite_ik_node
                        yaml-cpp urdf)

ament_package()
