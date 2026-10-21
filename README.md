# Lite_move_controller

Lite 双臂运动控制仓库，包含两个一起版本化的 ROS 2 包：

- `lite_move_controller`：运动控制节点、控制律、动力学前馈和示范脚本。
- `move_msgs`：控制器、IK 节点与上游目标源共享的消息和服务契约。

运动学实现与节点位于独立仓库
[`Lite_IK_node`](https://github.com/TeamLite-DeepCybo/Lite_IK_node)，动力学辨识位于
[`lite_dyid`](https://github.com/TeamLite-DeepCybo/lite_dyid)。三个仓库应并列放在
同一工作空间的 `src` 目录中：

```text
<workspace>/src/
├── Lite_move_controller/
│   ├── lite_move_controller/
│   └── move_msgs/
├── Lite_IK_node/
└── lite_dyid/
```

```bash
cd <workspace>/src
git clone https://github.com/TeamLite-DeepCybo/Lite_move_controller.git
git clone https://github.com/TeamLite-DeepCybo/Lite_IK_node.git
git clone https://github.com/TeamLite-DeepCybo/lite_dyid.git

cd ..
# ⚠️ 编译注意：必须使用系统 Python 环境编译，严禁在 Anaconda 环境下编译！
# 若处于 (base) 环境请先执行: conda deactivate
# 否则 Anaconda 的 yaml-cpp 0.9 会污染构建，导致运行时报 libyaml-cpp.so.0.9 找不到的致命错误。
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to lite_move_controller
source install/setup.bash
```

控制结构、默认动力学参数、启动方法和构型变更流程见
[`lite_move_controller/README.md`](lite_move_controller/README.md)。接口定义见
[`move_msgs/README.md`](move_msgs/README.md)。
