# EC130 Deploy

EC130 人形机器人 ROS2 Jazzy 部署包。

## 概述

本项目为 EC130 人形机器人提供完整的 ROS2 控制系统，包括：

- 关节电机和 IMU 传感器的硬件抽象层
- 遥控器通信（IBUS 协议）
- 三种控制模式：自由、站立、强化学习行走
- 基于 ONNX Runtime 的策略推理

## 软件包

| 软件包 | 说明 |
|--------|------|
| [ec_description](src/ec_description/) | EC130 机器人 URDF 模型和网格文件 |
| [ec_joint_hardware](src/ec_joint_hardware/) | 关节电机 ros2_control 硬件插件 |
| [ec_imu_hardware](src/ec_imu_hardware/) | IMU 传感器 ros2_control 硬件插件 |
| [ec_radio](src/ec_radio/) | 遥控器通信桥接节点（IBUS 协议） |
| [ec_controller](src/ec_controller/) | 控制器插件（默认/站立/行走），支持强化学习策略 |

## 环境要求

- Ubuntu 24.04
- ROS2 Jazzy

## 安装

```bash
# 安装依赖
sudo apt install -y \
  libyaml-cpp-dev \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-serial-driver \
  ros-jazzy-realtime-tools \
  ros-jazzy-asio-cmake-module \
  ros-jazzy-xacro

# 克隆仓库
git clone https://github.com/your-username/encos130_deploy.git
cd encos130_deploy

# 编译
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 启动控制器

```bash
ros2 launch ec_controller real.launch.py
```

### 启动遥控器节点

控制器启动后，需要单独启动遥控器节点：

```bash
ros2 run ec_radio radio_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

### 遥控器通道映射

遥控器使用 IBUS 协议，通道映射如下：

| 通道 | 功能 | 说明 |
|------|------|------|
| CH0 | wz | 偏航角速度（左右转向） |
| CH2 | vx | 前后速度 |
| CH3 | vy | 左右速度 |
| CH4 | mode=2 | 按键切换到站立模式 |
| CH5 | mode=3 | 按键切换到行走模式 |
| CH6 | mode=4 | 预留 |
| CH7 | mode=5 | 预留 |

**通道值范围**：1000-2000，中位 1500

**死区**：CH0/CH2/CH3 有 ±10% 死区（1450-1550 归零）

**模式切换**：CH4-CH7 为按键通道，按下时值 >1500，未按下时值 <1500，同时只有一个按键有效。无按键按下时默认为模式 1（电机自由）。

### 控制模式

| 模式 | 控制器 | 说明 |
|------|--------|------|
| 1 | default_controller | 零增益，电机自由 |
| 2 | stand_controller | 保持站立姿态 |
| 3 | walk_controller | 强化学习行走 |

### 手动切换控制模式

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['stand_controller'], deactivate_controllers: ['default_controller']}"
```

## 项目结构

```
encos130_deploy/
├── src/
│   ├── ec_description/          # URDF 模型
│   │   ├── meshes/              # STL 网格文件
│   │   └── urdf/                # URDF/XACRO 文件
│   ├── ec_joint_hardware/       # 关节硬件插件
│   │   ├── include/
│   │   ├── src/
│   │   └── test/
│   ├── ec_imu_hardware/         # IMU 硬件插件
│   │   ├── include/
│   │   ├── src/
│   │   └── test/
│   ├── ec_radio/                # 遥控器节点
│   │   ├── msg/                 # 自定义消息
│   │   └── src/
│   └── ec_controller/           # 控制器插件
│       ├── config/              # 配置文件
│       │   ├── controllers.yaml # 控制器参数
│       │   └── policy/          # RL 策略模型
│       ├── include/ec_controller/
│       ├── launch/              # 启动文件
│       └── src/
├── LICENSE
└── README.md
```

## 硬件配置

### 关节配置

关节配置文件：`src/ec_controller/config/controllers.yaml`

包含 23 个关节的参数：
- 左腿（6个）：hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
- 右腿（6个）：同上
- 腰部（3个）：yaw, roll, pitch
- 左臂（4个）：shoulder_pitch, shoulder_roll, shoulder_yaw, elbow
- 右臂（4个）：同上

### 策略模型

强化学习策略模型：`src/ec_controller/config/policy/policy.onnx`

- 输入：78 维观测向量（角速度、重力、命令、关节位置、关节速度、历史动作）
- 输出：23 维动作向量
- 推理频率：50Hz

## 许可证

GPL-3.0 - 详见 [LICENSE](LICENSE)
