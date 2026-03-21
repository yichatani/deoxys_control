# 快速开始指南

## 5分钟快速启动

### 1. 安装依赖

```bash
# Ubuntu 20.04/22.04
sudo apt-get update
sudo apt-get install -y \
    cmake \
    build-essential \
    python3-dev \
    python3-pip \
    python3-pybind11

# Python 依赖
pip3 install numpy
```

### 2. 安装 libfranka

**选项 A: 从 ROS (推荐)**
```bash
sudo apt-get install ros-noetic-libfranka
source /opt/ros/noetic/setup.bash
```

**选项 B: 从源码编译**
```bash
cd /tmp
git clone --recursive https://github.com/frankaemika/libfranka.git
cd libfranka
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

### 3. 编译项目

```bash
# 克隆或复制项目文件到目录
cd /path/to/franka_async_controller

# 编译
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release  -DCMAKE_PREFIX_PATH=/opt/openrobots
make -j$(nproc)

# 设置 Python 路径
export PYTHONPATH=$PYTHONPATH:$(pwd)
```

### 4. 测试连接

**Python 测试：**
```bash
python3 ../test_controller_fixed_rate.py 172.16.0.2
```

**C++ 测试：**
```bash
sudo setcap cap_sys_nice+ep ./franka_async_controller
sudo setcap cap_sys_nice+ep ./test_controller
./test_controller 172.16.0.2
```

## 最小代码示例

### Python

```python
import franka_controller as fc
import numpy as np
import time

# 连接
controller = fc.FrankaAsyncController("172.16.0.2")
controller.start()

# 获取当前位置
state = controller.get_robot_state()
current_q = state.q.copy()

# 移动
target = current_q + 0.05  # 每个关节 +0.05 rad
controller.set_joint_position_target(target)

# 等待
time.sleep(2)

# 停止
controller.stop()
```

### C++

```cpp
#include "franka_async_controller.h"

int main() {
    franka_control::FrankaAsyncController controller("172.16.0.2");
    controller.start();
    
    auto state = controller.getRobotState();
    std::array<double, 7> target = state->q;
    for(auto& q : target) q += 0.05;
    
    controller.setJointPositionTarget(target);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    controller.stop();
    return 0;
}
```

## 常见网络配置

### 检查连接

```bash
ping 172.16.0.2
```

### 设置静态IP（如果需要）

编辑 `/etc/netplan/01-network-manager-all.yaml`:

```yaml
network:
  version: 2
  ethernets:
    enp0s31f6:  # 你的网卡名
      dhcp4: no
      addresses:
        - 172.16.0.1/16
```

应用：
```bash
sudo netplan apply
```

### 防火墙（如果有问题）

```bash
sudo ufw allow from 172.16.0.0/16
```

## 故障排查

### 问题：无法连接机器人

**检查：**
```bash
ping 172.16.0.2
telnet 172.16.0.2 1337
```

**解决：**
- 检查网线连接
- 确认 IP 配置
- 检查防火墙设置

### 问题：编译错误 - 找不到 Franka

**错误信息：**
```
CMake Error: Could not find a package configuration file provided by "Franka"
```

**解决：**
```bash
# 检查安装
dpkg -l | grep franka

# 设置 CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH

# 或指定路径
cmake .. -DFranka_DIR=/path/to/libfranka/build
```

### 问题：Python 找不到模块

**错误信息：**
```python
ImportError: No module named 'franka_controller'
```

**解决：**
```bash
# 在 build 目录
export PYTHONPATH=$PYTHONPATH:$(pwd)

# 或安装
cd build
sudo make install
```

### 问题：控制器频率不稳定

**原因：** 实时性能问题

**解决：**
```bash
# 检查 CPU governor
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 设置为 performance
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 禁用节能
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

### 问题：机器人紧急停止

**可能原因：**
1. 碰撞检测触发
2. 关节限位
3. 速度/加速度过大

**解决：**
```python
# 降低速度限制
config = fc.ControllerConfig()
config.maximum_joint_velocities = [0.3, 0.3, 0.3, 0.3, 0.6, 0.6, 0.6]

# 或调整碰撞行为（在 C++ 中）
robot->setCollisionBehavior(
    {{30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0}},  // 增加阈值
    ...
)
```

## 性能优化建议

### 1. 实时性优化

```bash
# 设置 CPU 隔离
# 编辑 /etc/default/grub
GRUB_CMDLINE_LINUX="isolcpus=2,3"
sudo update-grub
sudo reboot

# 在代码中绑定线程到隔离的 CPU
```

### 2. 网络优化

```bash
# 增加网络缓冲区
sudo sysctl -w net.core.rmem_max=8388608
sudo sysctl -w net.core.wmem_max=8388608
```

### 3. Python 性能

```python
# 使用 numpy 数组而非列表
target = np.array([...])  # Good
target = [...]  # Slower

# 预分配数组
states = np.zeros((1000, 7))
for i in range(1000):
    states[i] = controller.get_robot_state().q
```

## 下一步

- 阅读 [README.md](README.md) 了解详细 API
- 查看 [test_controller_variable_rate.py](test_controller_variable_rate.py) 学习可变帧率控制
- 参考 [advanced_vision_control.py](advanced_vision_control.py) 了解视觉控制集成
- 学习 [diffusion_policy_control.py](diffusion_policy_control.py) 进行策略集成

## 获取帮助

如遇问题：
1. 检查本指南的故障排查部分
2. 查看 libfranka 文档：https://frankaemika.github.io/libfranka/
3. 检查机器人的 Web 界面：https://172.16.0.2
