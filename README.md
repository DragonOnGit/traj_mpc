# Traj MPC Package

基于ROS1的MPC轨迹跟踪控制功能包（无MAVROS依赖），使用QP优化求解器实现模型预测控制，采用一阶质点动力学模型。

## 功能特点

1. **MPC参数可调节**：通过launch文件或ROS参数服务器调整MPC控制器参数
2. **三维独立参数**：X/Y/Z轴方向的控制参数可单独配置，实现精细化控制
3. **QP优化求解**：集成二次规划(QP)优化方法作为MPC核心求解器
4. **一阶质点动力学模型**：基于牛顿第二定律的双积分器模型，直接以加速度为控制输入
5. **速度控制**：MPC计算最优加速度，转换为速度指令通过/cmd_vel话题发布
6. **重力补偿**：可配置的Z轴重力扰动模型，适配不同底层控制器
7. **单一指令发布**：每个控制周期仅发布一条速度指令，避免指令冲突
8. **XML轨迹定义**：期望轨迹由XML文件定义，关键点使用point标签表示
9. **四元数姿态**：轨迹点包含位置和姿态四元数信息
10. **无MAVROS依赖**：适用于各种机器人平台，不依赖特定的飞控系统
11. **参考轨迹发布**：将参考轨迹数据发布至/path_exp话题，便于可视化和调试
12. **期望位置发布**：将期望轨迹当前位置发布至/position_exp话题
13. **管道边界发布**：基于参考信号发布状态上下界到/tube_upperbound和/tube_lowerbound话题
14. **数据一致性验证**：自订阅/cmd_vel话题，检测多节点发布冲突

## 目录结构

```
traj_mpc/
├── CMakeLists.txt
├── package.xml
├── CHANGELOG.md               # 修改记录（独立于README）
├── src/
│   ├── traj_mpc_node.cpp      # 主节点（MPC-only速度控制）
│   ├── mpc_controller.cpp     # MPC控制器实现（一阶质点模型+QP求解器）
│   └── trajectory_loader.cpp  # 轨迹加载器实现
├── include/
│   └── traj_mpc/
│       ├── mpc_controller.h   # MPC控制器头文件
│       └── trajectory_loader.h # 轨迹加载器头文件
├── launch/
│   ├── traj_mpc.launch        # 启动文件
│   └── traj_mpc_with_recording.launch # 带录制的启动文件
├── trajectories/
│   └── example_trajectory.xml # 示例轨迹文件
└── README.md                  # 本说明文件
```

## 模型说明

系统采用一阶质点动力学模型（双积分器）：

```
p' = v
v' = a
```

其中：
- `p`：位置
- `v`：速度
- `a`：加速度（控制输入 = 合力/质量）

离散化后的状态空间方程：

```
[p(k+1)]   [I_3   dt*I_3] [p(k)]   [0.5*dt^2*I_3] [a(k)]   [d_p]
[v(k+1)] = [0_3   I_3   ] [v(k)] + [dt*I_3       ] [a(k)] + [d_v]
```

状态向量：`x = [px, py, pz, vx, vy, vz]`（6维，位置优先排列）
控制输入：`u = [ax, ay, az]`（3维）

当 `need_gravity_compensation=true` 时，重力扰动通过精确连续时间解直接嵌入预测矩阵：
```
D_vec[k] 中:
  delta_p_z(k) = -0.5 * g * (k+1)^2 * dt^2
  delta_v_z(k) = -g * (k+1) * dt
```
其中 `k` 为预测步索引（0到N-1），`(k+1)` 为从当前状态到该预测步的步数。

## MPC控制算法

### 目标函数

```
J = Σ_{k=1}^{N} [ (x(k) - x_ref(k))^T · Q̄ · (x(k) - x_ref(k)) ]
  + Σ_{k=0}^{N-1} a(k)^T · R · a(k)
```

其中 Q̄ 包含位置权重 P 和速度权重 Q。

### 状态空间提升与QP求解

预测方程（含扰动）：
```
X_pred = F · x₀ + G · U + D_vec
```

标准QP形式：
```
H = 2·(G^T·Q̄·G + R̄)
f = 2·G^T·Q̄·(F·x₀ + D_vec - X_ref)
```

约束条件：
- 加速度约束：`-max_acc ≤ a ≤ max_acc`（各轴独立）
- 速度约束：通过收紧第一步加速度边界实现

### 速度指令生成

```
v_cmd = v_current + a_cmd · dt
```

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
| weight_pos_z | 20.0 | Z轴位置误差权重 |

### 三维速度权重（Q矩阵对角元素）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| weight_vel_x | 1.0 | X轴速度权重 |
| weight_vel_y | 1.0 | Y轴速度权重 |
| weight_vel_z | 0.5 | Z轴速度权重 |

### 三维加速度权重（R矩阵对角元素）

| 参数 | 默认值 | 描述 |
|------|--------|------|
| weight_acc_x | 0.1 | X轴加速度权重 |
| weight_acc_y | 0.1 | Y轴加速度权重 |
| weight_acc_z | 0.05 | Z轴加速度权重 |

### 三维速度约束

| 参数 | 默认值 | 描述 |
|------|--------|------|
| max_vel_x | 2.0 | X轴最大速度 (m/s) |
| max_vel_y | 2.0 | Y轴最大速度 (m/s) |
| max_vel_z | 2.0 | Z轴最大速度 (m/s) |

### 三维加速度约束

| 参数 | 默认值 | 描述 |
|------|--------|------|
| max_acc_x | 5.0 | X轴最大加速度 (m/s²) |
| max_acc_y | 5.0 | Y轴最大加速度 (m/s²) |
| max_acc_z | 10.0 | Z轴最大加速度 (m/s²) |

**注意**：`max_acc_z` 必须大于重力加速度(9.81 m/s²)才能实现垂直起飞。

### 重力参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| gravity | 9.81 | 重力加速度 (m/s²) |
| need_gravity_compensation | false | 是否在MPC模型中包含重力扰动 |

**need_gravity_compensation说明**：
- `false`（默认）：底层速度控制器已包含重力补偿（如PX4、ArduPilot的定高模式）
- `true`：底层速度控制器不包含重力补偿，MPC模型中添加重力扰动向量

### 管道边界参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| tube_bound_radius | 0.3 | 管道边界半径（米） |

## 话题配置参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| odom_topic | /odom | 里程计话题名称 |
| cmd_vel_topic | /cmd_vel | 速度控制指令话题名称 |
| path_topic | /path_exp | 参考轨迹发布话题名称 |
| position_exp_topic | /position_exp | 期望位置发布话题名称 |
| tube_upperbound_topic | /tube_upperbound | 状态上界话题名称 |
| tube_lowerbound_topic | /tube_lowerbound | 状态下界话题名称 |
| trajectory_file | trajectories/example_trajectory.xml | 轨迹文件路径 |

## 话题列表

| 话题名称 | 消息类型 | 方向 | 频率 | 描述 |
|---------|---------|------|------|------|
| /odom | nav_msgs/Odometry | 订阅 | - | 里程计输入 |
| /cmd_vel | geometry_msgs/Twist | 发布+自订阅 | 10Hz | 速度控制指令 |
| /path_exp | nav_msgs/Path | 发布 | 10Hz | 参考轨迹 |
| /position_exp | nav_msgs/Odometry | 发布 | 10Hz | 期望位置 |
| /tube_upperbound | nav_msgs/Odometry | 发布 | 10Hz | 状态上界 |
| /tube_lowerbound | nav_msgs/Odometry | 发布 | 10Hz | 状态下界 |

## 控制模式

系统使用**MPC-only速度控制模式**进行轨迹跟踪：

1. **MPC计算**：使用一阶质点模型，通过状态空间提升和QP求解器计算最优加速度序列
2. **加速度转速度**：`v_cmd = v_current + a_cmd · dt`
3. **安全限幅**：速度指令被限制在`max_vel`范围内
4. **单一发布**：每个控制周期仅发布一条速度指令到`/cmd_vel`
5. **数据验证**：自订阅`/cmd_vel`，检测多节点发布冲突

## 启动方式

```bash
roslaunch traj_mpc traj_mpc.launch
```

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

## 修改记录

详细的修改记录请参见 [CHANGELOG.md](CHANGELOG.md)。

## 联系方式

如有问题或建议，请联系：xiaolongw@nuaa.edu.cn
