# 故障注入与异常恢复验证 Runbook (Fault Injection & Recovery Runbook)

> **⚠️ 极重要安全警示（前置强制确认）**
> 
> 在执行本文档中的任何故障注入或延迟测试之前，**必须**严格满足以下安全条件：
> 1. **人员与物理环境隔离**：确保双臂全部活动半径内严禁站人，清理机械臂周围的障碍物与易碎设备，确保机械结构即使在极端失控或重力下坠时也不会伤人或撞击硬物。
> 2. **具备可靠的急停与交权途径**：测试前必须验证急停手段处于随时待命状态：
>    - 软件急停终端待命：预备一个单独终端，随时准备执行交权/失能指令：
>      ```bash
>      ros2 service call /lite_move_controller/disable std_srvs/srv/Trigger "{}"
>      # 或强制终止运控进程：
>      pkill -9 -f "move_controller_node|lite_ik_node"
>      ```
> 3. **随时做好硬件级断电准备**：在测试通信掉线与进程挂起期间，底层电机可能因超时进入零力矩/阻尼保护状态，机械臂会在重力下发生瞬间快速下坠。**操作人员的手必须随时放在系统总电源开关/急停拍钮上，一旦发生异常碰撞、飞车、堵转过流或异响，立即拍下物理总电源断电！**

---

## 一、 测试目的与原理

本 Runbook 用于在不侵入修改系统源码的前提下，通过 Linux POSIX 信号与 ROS 2 标准接口模拟嵌入式控制器在实际作业中可能遇到的偶发异常：
- **进程调度饥饿 / CPU 卡死阻塞 (`SIGSTOP` / `SIGCONT`)**
- **上游规划链路断流与丢包超时 (`target_gap_timeout`)**
- **外力拉扯或电机掉电坠落导致的关节大偏差 (`drift_threshold`)**
- **服务接口与参数模式热切换**

测试将检验 `lite_ik_node`（五次同伦混合过渡桥与双模式状态机）与 `lite_move_controller`（断线基准同步与反转限幅）能否在故障消除后实现**零单步突跳、无自激振荡、1.2秒内平滑重捕获轨迹**。

---

## 二、 前置启动准备

在**终端 1**正常拉起机械臂运动控制与左圆右方测试任务：

```bash
# 1. 确保 bringup 已启动
# 2. 启动圆-方轨迹测试（建议适当放宽测试时长，如 200s）
DURATION=200 SPEED=0.05 bash src/Lite_move_controller/lite_move_controller/scripts/run_circle_square.sh
```

---

## 三、 单项精准故障注入测试命令

在机械臂处于正常跟踪状态时，在**终端 2**中按需执行以下命令：

### 1. 复现真机丢帧事故（IK 节点短时卡顿 350 ms）
* **模拟场景**：完全复原嵌入式设备上出现的 348 ms 调度停顿。
* **执行命令**：
  ```bash
  pkill -STOP -f lite_ik_node && sleep 0.35 && pkill -CONT -f lite_ik_node
  ```
* **验收准则**：机械臂因短时失步受重力微幅下沉；唤醒瞬间 `lite_ik_node` 触发 `RECOVERY` 模式，以当前物理实测位姿无缝平滑起步，并在 1.2 秒内平滑重返轨迹，**绝无剧烈晃动或 10 Hz 抽搐自激振荡**。

### 2. 极限重力下坠平滑重捕获（IK 节点挂起 800 ms）
* **模拟场景**：长时间严重阻塞，机械臂大幅度下落（肘部下垂 $> 45^\circ$）。
* **执行命令**：
  ```bash
  pkill -STOP -f lite_ik_node && sleep 0.8 && pkill -CONT -f lite_ik_node
  ```
* **验收准则**：恢复后的首帧下发指令完全对齐下坠实测位置（偏差为 0），随后以 $C^2$ 连续速度平稳将机械臂托举归位并自动切回 `NORMAL` 模式。

### 3. 上层规划断流测试（规划器卡死 600 ms）
* **模拟场景**：上游相机掉帧、算法阻塞或 `/task_target` 传输中断。
* **执行命令**：
  ```bash
  pkill -STOP -f run_circle_square.py && sleep 0.6 && pkill -CONT -f run_circle_square.py
  ```
* **验收准则**：IK 节点检测到数据断帧超过 150 ms 门限，上游恢复发帧时自动转入平滑捕获。

### 4. 运控层线程挂起测试（Move Controller 挂起 400 ms）
* **模拟场景**：测试 `move_controller_node` 超时后的基准重置与步长硬限幅。
* **执行命令**：
  ```bash
  pkill -STOP -f move_controller_node && sleep 0.4 && pkill -CONT -f move_controller_node
  ```
* **验收准则**：运控恢复后的第一拍命令不得出现历史位置与当前实测间的 0.2 rad 突跳。

### 5. 远程服务手动注入测试
* **模拟场景**：外部监控节点主动触发机械臂同伦重捕获。
* **执行命令**：
  ```bash
  ros2 service call /lite_ik_node/trigger_recovery std_srvs/srv/Trigger "{}"
  ```
* **验收准则**：服务返回成功，机械臂当前正在运动的臂以当前点为起点执行一次平滑重新锚定。

---

## 四、 自动化随机“混沌猴”注入器 (Chaos Monkey)

若需在长周期测试中进行无人值守的多故障混合极限压力测试，可以在**终端 2**运行以下独立 Python 注入脚本：

```bash
/usr/bin/python3 -c "
import time, random, subprocess

faults = [
    ('IK 节点瞬时卡顿 (复现真机丢帧)', 'lite_ik_node', 0.25, 0.45),
    ('IK 节点深度挂起 (测试大幅坠落恢复)', 'lite_ik_node', 0.60, 0.90),
    ('上游规划断帧 (测试目标断流)', 'run_circle_square.py', 0.30, 0.70),
    ('运控层调度挂起 (测试基准同步)', 'move_controller_node', 0.20, 0.40),
    ('远程服务注入 (主动触发 Recovery)', 'SERVICE', 0, 0)
]

print('\033[1;32m=== 机械臂随机故障注入器启动 (按 Ctrl+C 停止) ===\033[0m')
try:
    while True:
        wait_s = random.uniform(6.0, 12.0)
        print(f'\n[等待] {wait_s:.1f} 秒后发起下一次随机故障注入...')
        time.sleep(wait_s)

        name, target, min_t, max_t = random.choice(faults)
        if target == 'SERVICE':
            print(f'\033[1;33m[注入故障] -> {name}\033[0m')
            subprocess.run(['ros2', 'service', 'call', '/lite_ik_node/trigger_recovery', 'std_srvs/srv/Trigger', '{}'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            duration = random.uniform(min_t, max_t)
            print(f'\033[1;31m[注入故障] -> {name} | 阻塞挂起: {duration*1000:.0f} ms\033[0m')
            subprocess.run(['pkill', '-STOP', '-f', target])
            time.sleep(duration)
            subprocess.run(['pkill', '-CONT', '-f', target])
        print('\033[1;32m[注入结束] 进程已唤醒，请观察机械臂是否平滑恢复轨迹。\033[0m')
except KeyboardInterrupt:
    print('\n[安全退出] 确保所有进程已恢复 CONT...')
    for _, t, _, _ in faults:
        if t != 'SERVICE': subprocess.run(['pkill', '-CONT', '-f', t], stderr=subprocess.DEVNULL)
    print('退出完成。')
"
```

---

## 五、 状态与指标验收监控

在测试进行时，可在**终端 3**打开日志监控观察状态流转：

```bash
# 查看 IK 节点的实时状态切换与警告日志
ros2 node info /lite_ik_node
# 监控终端实时输出中的平滑切换特征：
#   [WARN] 臂 'left' 触发 RECOVERY 恢复模式：(drift=... rad, gap_timeout=...)，开启五次同伦混合平滑过渡 (时长 1.20 s)
#   [INFO] 臂 'left' 成功完成 RECOVERY 过渡，已平滑并入主轨迹并切回 NORMAL 模式
```
* **合格指标**：
  1. 无论注入何种卡顿，单拍关节最大位置增量绝对不得超过限幅门限（$\Delta q \le 0.05\text{ rad}$）；
  2. 机械臂在任何情况下不得发生高频往复摆动或碰撞保护报警。
