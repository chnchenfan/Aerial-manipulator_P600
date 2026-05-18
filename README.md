# P600 空中机械臂 PX4-ESO 控制仓库

## 1. 摘要

本仓库是 P600 空中机械臂平台的 PX4-ESO 飞控实现仓库，负责在 `codev_dp1000-v2` 飞控硬件上执行基于扩张状态观测器（ESO）的无人机位置、姿态和角速度控制。与配套仓库 [`chnchenfan/p600_arm_control`](https://github.com/chnchenfan/p600_arm_control) 和 [`chnchenfan/P600_uav_control`](https://github.com/chnchenfan/P600_uav_control) 配合后，可完成真实平台上的联合飞行与机械臂实验。

当前 README 仅覆盖两组真实实验：

- `exp1`，在本文中记为 `mode1`
- `exp3`，其启动脚本名为 `exp3`，但当前 `data_P600` 内的图和 `.mat` 文件沿用 `mode2` 命名

当前提交的实飞结果表明，本文算法在两组实验中的平均位置误差分别为：

- `exp1 / mode1`：`0.085953 m`
- `exp3 / mode2(data_P600命名)`：`0.082433 m`

如果以我的本科毕设的 `0.1 m` 平均位置误差指标作为参考，这两组结果都已经达到或略优于该指标。对应的位置均方根误差分别为：

- `exp1 / mode1`：`0.110683 m`
- `exp3 / mode2(data_P600命名)`：`0.097174 m`

实飞数据和图表位于 [`data_P600/`](data_P600)。

## 2. 项目结构

本项目的真实实验链路由三个仓库组成：

- 当前仓库 [`Codev-autopilot`](.)：PX4-ESO 飞控实现与固件构建
- [`chnchenfan/P600_uav_control`](https://github.com/chnchenfan/P600_uav_control)：MAVROS/PX4 执行链、定位接入、期望飞行执行
- [`chnchenfan/p600_arm_control`](https://github.com/chnchenfan/p600_arm_control)：机械臂控制、实验期望生成、联合实验启动脚本、录包与绘图

### 2.1 当前仓库：PX4-ESO 飞控层

本仓库主要关注 PX4 固件内部的 ESO 控制链：

```text
src/modules/eso_pos_control     位置/速度 ESO 控制
src/modules/eso_att_control     姿态 ESO 控制
src/modules/eso_rate_control    角速度 ESO 控制
src/modules/eso_common          共享模型与参数
src/modules/arm_joint_bridge    机械臂关节桥接
msg/                            自定义 uORB/msg 接口
ROMFS/.../4066_codev_dp_1000_eso  P600/DP1000 ESO airframe
data_P600/                      实飞数据、导出脚本、MATLAB 绘图和结果图
```


## 3. 工作流程和使用情况

### 3.1 编译固件

本仓库负责编译 P600 平台使用的 PX4-ESO 固件。编译命令为：

```bash
cd /home/cf/Program/code/Codev-autopilot/Codev-autopilot
make codev_dp1000-v2_default
```

生成的固件文件位于：

```text
build/codev_dp1000-v2_default/codev_dp1000-v2_default.px4
```

刷写后可在飞控中启用本仓库提供的 ESO airframe：

```sh
param set SYS_AUTOSTART 4066
param save
reboot
```

### 3.2 `data_P600` 绘图命令

当前 `data_P600/scripts/plot_px4_pid_comparison.m` 的函数接口为：

```matlab
plot_px4_pid_comparison(mode1_eso_file, mode2_eso_file, output_dir)
```

在本仓库中直接绘图可使用：

```matlab
cd('/home/cf/Program/code/Codev-autopilot/Codev-autopilot')
addpath('data_P600/scripts')
plot_px4_pid_comparison( ...
    'data_P600/raw/p600_mode1_exp1_paper_eso.mat', ...
    'data_P600/raw/p600_mode2_exp3_paper_eso.mat', ...
    'data_P600/figure')
```

其中：

- `p600_mode1_exp1_paper_eso.mat` 对应 `exp1 / mode1`
- `p600_mode2_exp3_paper_eso.mat` 对应 `exp3 / mode2(data_P600命名)`

绘图输出文件包括：

- `data_P600/figure/mode1_position_tracking_and_error_p600_real.png`
- `data_P600/figure/mode1_arm_tracking_p600_real.png`
- `data_P600/figure/mode1_3d_mean_error_p600_real.png`
- `data_P600/figure/mode2_position_tracking_and_error_p600_real.png`
- `data_P600/figure/mode2_arm_tracking_p600_real.png`
- `data_P600/figure/mode2_3d_mean_error_p600_real.png`

指标汇总文件为：

- `data_P600/figure/p600_real_metrics_summary.txt`

## 4. 验证场景

### 4.1 `mode1 / exp1`：悬停扰动实验

该场景下，无人机保持近似固定悬停点，机械臂执行周期运动，用于观察机械臂扰动对基座位置保持能力的影响。当前提交结果：

- 平均位置误差：`0.085953 m`
- 各轴平均位置误差：`[0.038743 0.053022 0.033326] m`
- 位置均方根误差：`0.110683 m`
- 最大位置误差：`0.572978 m`

相关数据和图表位于：

- `data_P600/raw/p600_mode1_exp1_paper_eso.mat`
- `data_P600/figure/mode1_position_tracking_and_error_p600_real.png`
- `data_P600/figure/mode1_arm_tracking_p600_real.png`
- `data_P600/figure/mode1_3d_mean_error_p600_real.png`

### 4.2 `mode2 / exp3`：方形轨迹跟踪实验

该场景下，无人机跟踪方形轨迹，机械臂同步执行周期运动，用于观察轨迹跟踪与耦合扰动下的整体控制效果。当前提交结果：

- 平均位置误差：`0.082433 m`
- 各轴平均位置误差：`[0.031051 0.063812 0.021942] m`
- 位置均方根误差：`0.097174 m`
- 最大位置误差：`0.514198 m`

相关数据和图表位于：

- `data_P600/raw/p600_mode2_exp3_paper_eso.mat`
- `data_P600/figure/mode2_position_tracking_and_error_p600_real.png`
- `data_P600/figure/mode2_arm_tracking_p600_real.png`
- `data_P600/figure/mode2_3d_mean_error_p600_real.png`

### 4.3 结果波动说明

`data_P600` 中保存的是当前提交版本下的一组真实实验结果。由于真实环境中的模型参数、初始状态、外部扰动、定位噪声、气流和执行器状态不可能每次完全一致，因此即使使用同一套代码和参数，重复运行时得到的数据也可能存在毫米级到厘米级差异。

这类差异属于真实平台实验中的正常现象。阅读和比较结果时，应重点关注：

- 平均误差和 RMSE 是否保持在同一数量级
- 误差峰值是否出现在相近阶段
- 整体轨迹是否稳定、是否出现发散或明显饱和

当前提交的两组结果中：

- `is_divergent = 0`
- `thrust_saturation_ratio = 0`
- `torque_saturation_ratio = 0`
