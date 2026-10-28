# move_msgs

`Lite_move_controller` 与上游目标源之间的 ROS 2 接口包，随
`Lite_move_controller` 仓库一起发布。

## 接口

- `MoveTarget` / `MoveTargetArray`：末端位姿或关节目标。
- `MoveStatus`：控制状态、IK 结果、关节命令与诊断计数。
- `EeResidual`：目标与当前可达点之间的末端残差。
- `InteractionEvent`：接触与外力估计事件。
- `QueryIk`：不下发命令的可达性查询。
- `GetMoveStatus`：按请求读取当前控制器状态。

该包只依赖 ROS 标准消息。它保持独立 ROS 包名，使 `lite_ik_node` 和
`lite_move_controller` 可以共同使用接口而不形成循环依赖。

```bash
colcon build --packages-select move_msgs
```

`MoveTarget.gripper` 为兼容保留的弃用字段，关节和位姿路径都不消费它。
夹爪请使用 `/gripper/command`（JointState，按名称指定 0～1 开度）；
该新接口的显式 0 表示闭合，省略侧保持目标。
