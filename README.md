# Traj MPC Package

基于ROS1的MPC轨迹跟踪控制功能包（无MAVROS依赖），使用QP优化求解器实现模型预测控制。

## 功能特点

1. **MPC参数可调节**：通过launch文件或ROS参数服务器调整MPC控制器参数
2. **三维独立参数**：X/Y/Z轴方向的控制参数可单独配置，实现精细化控制
3. **QP优化求解**：集成二次规划(QP)优化方法作为MPC核心求解器
4. **二阶动力学模型**：基于二阶系统动力学模型进行状态空间方程推导
5. **速度控制**：控制指令使用速度控制，通过/cmd_vel话题发布
6. **XML轨迹定义**：期望轨迹由XML文件定义，关键点使用point标签表示
7. **四元数姿态**：轨迹点包含位置和姿态四元数信息
8. **无MAVROS依赖**：适用于各种机器人平台，不依赖特定的飞控系统
9. **参考轨迹发布**：将参考轨迹数据发布至/path_exp话题，便于可视化和调试
10. **期望位置发布**：将期望轨迹当前位置发布至/position_exp话题

## 目录结构

```
traj_mpc/
├── CMakeLists.txt
├── package.xml
├── src/
│   ├── traj_mpc_node.cpp      # 主节点
│   ├── mpc_controller.cpp     # MPC控制器实现（含QP求解器）
│   └── trajectory_loader.cpp  # 轨迹加载器实现
├── include/
│   └── traj_mpc/
│       ├── mpc_controller.h   # MPC控制器头文件
│       └── trajectory_loader.h # 轨迹加载器头文件
├── launch/
│   └── traj_mpc.launch        # 启动文件
├── trajectories/
│   └── example_trajectory.xml # 示例轨迹文件
└── README.md                  # 本说明文件
```

## MPC控制算法

### 目标函数

MPC优化的目标函数为：

```
J = e^T P e + v^T Q v
```

其中：
- `e` = p_ref - p_current - v·dt：当前位置误差向量（三维），表示施加速度v经过dt后的预测位置误差
- `v`：当前速度向量（三维）
- `P`：位置误差权重矩阵（3×3对角阵），对角元素为[weight_pos_x, weight_pos_y, weight_pos_z]
- `Q`：速度权重矩阵（3×3对角阵），对角元素为[weight_vel_x, weight_vel_y, weight_vel_z]

### QP优化求解

将目标函数展开为标准QP形式：

```
J = (e0 - v·dt)^T P (e0 - v·dt) + v^T Q v
  = v^T (dt²P + Q) v - 2·dt·e0^T P v + const
```

标准QP形式：`min 0.5·v^T H v + f^T v`

其中：
- `H = 2·(dt²·P + Q)`
- `f = -2·dt·P·e0`

约束条件：
- 速度约束：`-max_vel ≤ v ≤ max_vel`（各轴独立）
- 加速度约束：`v_current - max_acc·dt ≤ v ≤ v_current + max_acc·dt`（各轴独立）

### 二阶系统动力学模型

系统采用二阶动力学模型：

```
x'' + 2·ζ·ω·x' + ω²·x = ω²·u
```

其中：
- `x`：位置
- `ζ`：阻尼比（damping）
- `ω`：自然频率（omega）
- `u`：控制输入

离散化后的状态空间方程（零阶保持）：

```
[p(k+1)]   [1          dt       ] [p(k)]   [0        ] [a(k)]
[v(k+1)] = [-ω²·dt    1-2·ζ·ω·dt] [v(k)] + [ω²·dt   ] [a(k)]
```

状态向量：`x = [p, v]^T`，输入：`u = a`（加速度）

### 多步预测

对于预测步长N > 1的情况，构建全时域QP问题：

- 决策变量：`[v_0, v_1, ..., v_{N-1}]`（3N维）
- 位置预测：`p(k) = p_current + dt·Σ(v_i), i=0..k`
- 误差：`e(k) = p_ref(k) - p_current - dt·Σ(v_i), i=0..k`

## MPC参数配置

### 基本参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| horizon | 10 | MPC预测步长 |
| dt | 0.1 | 时间步长（秒） |
| control_rate | 10.0 | 控制频率（Hz） |

### 三维位置误差权重（P矩阵对角元素）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| weight_pos_x | 10.0 | X轴位置误差权重 |
| weight_pos_y | 10.0 | Y轴位置误差权重 |
| weight_pos_z | 10.0 | Z轴位置误差权重 |

**调节建议**：
- 较大的值：优先保证该轴的位置跟踪精度
- 较小的值：允许较大的位置误差，系统更稳定
- 不同轴可设置不同值以适应不同方向的精度需求

### 三维速度权重（Q矩阵对角元素）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| weight_vel_x | 1.0 | X轴速度权重 |
| weight_vel_y | 1.0 | Y轴速度权重 |
| weight_vel_z | 1.0 | Z轴速度权重 |

**调节建议**：
- 较大的值：速度变化更平滑，减少抖动
- 较小的值：速度响应更快速，跟踪更灵敏

### 三维加速度权重（R矩阵对角元素）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| weight_acc_x | 0.1 | X轴加速度权重 |
| weight_acc_y | 0.1 | Y轴加速度权重 |
| weight_acc_z | 0.1 | Z轴加速度权重 |

### 三维速度约束

| 参数 | 默认值 | 描述 |
|------|--------|------|
| max_vel_x | 2.0 | X轴最大速度 (m/s) |
| max_vel_y | 2.0 | Y轴最大速度 (m/s) |
| max_vel_z | 2.0 | Z轴最大速度 (m/s) |

### 三维加速度约束

| 参数 | 默认值 | 描述 |
|------|--------|------|
| max_acc_x | 1.0 | X轴最大加速度 (m/s²) |
| max_acc_y | 1.0 | Y轴最大加速度 (m/s²) |
| max_acc_z | 1.0 | Z轴最大加速度 (m/s²) |

### 二阶模型参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| damping_x | 0.8 | X轴阻尼比 ζ |
| damping_y | 0.8 | Y轴阻尼比 ζ |
| damping_z | 0.8 | Z轴阻尼比 ζ |
| omega_x | 5.0 | X轴自然频率 ω (rad/s) |
| omega_y | 5.0 | Y轴自然频率 ω (rad/s) |
| omega_z | 5.0 | Z轴自然频率 ω (rad/s) |

**调节建议**：
- ζ < 1：欠阻尼，系统有超调但响应快
- ζ = 1：临界阻尼，无超调且响应最快
- ζ > 1：过阻尼，无超调但响应慢
- ω越大，系统响应越快，但可能引起振荡

## 话题配置参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| odom_topic | /odom | 里程计话题名称 |
| setpoint_pos_topic | /target_position | 目标位置话题名称（已弃用） |
| cmd_vel_topic | /cmd_vel | 速度控制指令话题名称 |
| path_topic | /path_exp | 参考轨迹发布话题名称 |
| position_exp_topic | /position_exp | 期望位置发布话题名称 |
| trajectory_file | trajectories/example_trajectory.xml | 轨迹文件路径 |

## 参考轨迹发布

- **话题名称**: `/path_exp`
- **消息类型**: `nav_msgs/Path`
- **发布频率**: 10Hz
- **数据格式**: 包含轨迹中所有路标点的PoseStamped消息数组
- **坐标系**: camera_init

## 期望位置发布

- **话题名称**: `/position_exp`
- **消息类型**: `nav_msgs/Odometry`
- **发布频率**: 与控制频率一致（默认10Hz）
- **数据格式**: 当前期望轨迹点的位置和姿态信息
- **坐标系**: camera_init
- **发布逻辑**:
  - 初始化时发布首个路标点
  - 路标点更新时立即发布新点
  - 未更新时持续发布当前点

## 控制模式

系统使用**速度控制模式**进行轨迹跟踪：

1. **MPC+QP控制**：使用模型预测控制算法，通过QP求解器计算最优速度指令
2. **速度控制**：直接发布速度指令到`/cmd_vel`话题
3. **位置控制**：已弃用，相关代码保留为注释

## 启动方式

```bash
roslaunch traj_mpc traj_mpc.launch
```

## 程序执行流程

1. **初始化阶段**：
   - 加载MPC参数（三维独立参数）和话题配置
   - 构建权重矩阵P、Q、R和状态空间矩阵A、B
   - 加载XML轨迹文件
   - 初始化订阅器和发布器
   - 进入IDLE状态，等待里程计数据

2. **轨迹跟踪阶段**：
   - 接收到第一条里程计消息后，自动开始轨迹跟踪
   - 构建QP问题：计算H矩阵和f向量
   - 通过QP求解器求解最优速度指令
   - 发布速度控制指令和期望位置

3. **任务完成阶段**：
   - 所有路标点访问完成后，进入COMPLETED状态
   - 发布零速度指令停止运动

## 轨迹文件格式

轨迹文件使用XML格式，示例如下：

```xml
<?xml version="1.0"?>
<trajectory>
  <point x="0.0" y="0.0" z="1.0" qx="0.0" qy="0.0" qz="0.0" qw="1.0" />
  <point x="1.0" y="0.0" z="1.0" qx="0.0" qy="0.0" qz="0.0" qw="1.0" />
</trajectory>
```

## 依赖项

- ROS Kinetic或更高版本
- Eigen3
- tinyxml2（用于XML解析）
- tf2_ros（用于坐标变换）

## 编译运行方法

```bash
cd ~/catkin_ws
catkin build

source devel/setup.bash
roslaunch traj_mpc traj_mpc.launch
```

## 联系方式

如有问题或建议，请联系：xiaolongw@nuaa.edu.cn
