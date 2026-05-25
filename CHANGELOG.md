# Changelog

本文档记录 traj_mpc 功能包的所有重要变更。

## [2.4] - 2026-05-06

### 修复
- **Z轴稳态误差0.510m**：修复重力扰动在MPC预测矩阵中的计算方式
  - **问题**：旧实现使用离散扰动向量 `d_` 通过 `Σ A^j * d_` 累积重力效应，导致多步预测时重力被过度补偿，MPC输出的加速度不足以对抗实际重力，产生0.510m的稳态位置偏差
  - **修复**：改为使用连续时间精确解直接计算每步预测的重力影响：`delta_p_z = -0.5*g*(k+1)^2*dt^2`，`delta_v_z = -g*(k+1)*dt`
  - **影响范围**：`mpc_controller.cpp` 的 `buildStateSpaceModel()` 和 `buildPredictionMatrices()` 方法
  - **预期效果**：启用重力补偿时，Z轴稳态位置误差从0.510m降低至±0.05m以内

## [2.3] - 2026-05-06

### 移除
- **移除干扰信号生成功能**：删除 `computeDisturbance()` 方法及所有相关代码
- **移除干扰注入逻辑**：删除 `traj_mpc_node.cpp` 中干扰信号叠加到速度指令的代码
- **移除干扰参数**：从 `MPCParams` 结构体中删除 `enable_disturbance`、`disturbance_amplitude`、`disturbance_frequency` 三个字段
- **移除干扰参数加载**：删除 `mpc_controller.cpp` 中 `enable_disturbance`、`disturbance_amplitude`、`disturbance_frequency` 的参数读取和日志输出
- **移除 launch 文件干扰配置**：删除 `traj_mpc.launch` 中的 Disturbance Signal Parameters 配置段
- **移除 `start_time_` 成员**：删除节点中仅用于干扰计算的时间戳变量

### 新增
- **创建 CHANGELOG.md**：将修改记录从 README 中独立出来

## [2.2] - 2026-04-29

### 新增
- **cmd_vel 自订阅验证**：新增 `cmd_vel_echo_sub_` 订阅器，回读 `/cmd_vel` 话题数据
- **数据一致性检测**：对比发布值与回读值，差异超过0.01时输出 `ROS_WARN` 告警
- **多节点发布冲突检测**：检测到数据不匹配时，自动提示可能有其他节点在发布
- **angular 字段显式初始化**：`cmd_vel.angular.x/y/z` 显式设为0.0，防止未初始化内存值
- **发布计数器**：新增 `publish_count_` 跟踪指令发布次数

## [2.1] - 2026-04-29

### 新增
- **管道边界话题发布**：新增 `/tube_upperbound` 和 `/tube_lowerbound` 话题（nav_msgs/Odometry）
- **管道边界参数化**：新增 `tube_bound_radius` 参数（默认0.3m），集成随机数生成器
- **干扰信号生成功能**：新增 `computeDisturbance()` 方法，实现时间相关正弦波干扰
- **干扰参数化控制**：新增 `enable_disturbance`、`disturbance_amplitude`、`disturbance_frequency` 参数
- **录制话题更新**：`traj_mpc_with_recording.launch` 新增管道边界话题录制

## [2.0] - 2026-04-28

### 变更
- **模型重构**：从二阶弹簧-阻尼系统改为一阶质点动力学模型（双积分器）
- **状态空间矩阵重构**：`A_full_`/`B_full_` 改为位置优先块矩阵格式，匹配 `[px,py,pz,vx,vy,vz]` 排序
- **重力扰动模型**：将重力作为MPC模型内的恒定扰动向量 `d_`，通过 `D_vec_` 累积到预测方程
- **参数变更**：移除 `damping_x/y/z`、`omega_x/y/z`；新增 `gravity`、`need_gravity_compensation`

### 修复
- **弹跳问题**：移除P控制器，实现单一MPC指令发布，解决双重指令冲突
- **反向运动**：修复状态空间矩阵与状态向量排序不匹配问题
- **重力补偿失控上冲**：将外部累积式速度增量改为MPC模型内部扰动
- **参考轨迹生成错误**：修复所有MPC预测步使用同一目标航点，避免冲突目标导致零加速度
- **Z轴高度跟踪不足**：`max_acc_z` 从3.0提高到10.0（需大于重力9.81），调整Z轴权重参数

### 新增
- **10Hz详细控制台输出**：包含位置误差、速度、加速度、权重、约束等完整信息
