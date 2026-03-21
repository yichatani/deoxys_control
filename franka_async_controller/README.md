# Used for IK and FK

## 编译安装

### 依赖项

```bash
# 安装 Franka library (libfranka)
sudo apt-get install ros-noetic-libfranka  # 如果使用 ROS
# 或从源码编译：https://github.com/frankaemika/libfranka

# 安装 pybind11
sudo apt-get install python3-pybind11
# 或
pip install pybind11

# 其他依赖
sudo apt-get install cmake build-essential
```

### 编译步骤

```bash
# 创建构建目录
mkdir build && cd build

# 配置 CMake
cmake .. -DCMAKE_BUILD_TYPE=Release  -DCMAKE_PREFIX_PATH=/opt/openrobots

# 编译
make -j$(nproc)

# 安装 (可选)
sudo make install
```

编译后会生成：
- `franka_controller.so` - Python 模块
- `libfranka_async_controller_lib.so` - C++ 库
- `test_controller` - C++ 测试程序

### 手动设置 Python 路径

如果不想安装，可以直接使用：

```bash
# 在 build 目录中
export PYTHONPATH=$PYTHONPATH:$(pwd)

# 或在 Python 中
import sys
sys.path.insert(0, '/path/to/build')
import franka_controller
```

## 使用示例

### Python - 基础使用

```python
import franka_controller as fc
import numpy as np
import time

# 配置控制器
config = fc.ControllerConfig()
config.verbose = True
config.control_period_ms = 20  # 50Hz 底层控制

# 创建并启动控制器
controller = fc.FrankaAsyncController("172.16.0.2", config)
controller.start()

# 获取当前状态
state = controller.get_robot_state()
if state is not None:
    print(f"Current position: {state.q}")
    initial_q = state.q.copy()

# 发送目标位置 (可以以任意帧率调用)
target = initial_q + 0.05
controller.set_joint_position_target(target)

# 等待运动完成
time.sleep(2.0)

# 停止
controller.stop()
```

### Python - 固定帧率控制

```python
# 10Hz 控制循环
rate = 10  # Hz
dt = 1.0 / rate

while running:
    loop_start = time.time()
    
    # 生成目标
    target = compute_target()
    controller.set_joint_position_target(target)
    
    # 保持固定帧率
    elapsed = time.time() - loop_start
    time.sleep(max(0, dt - elapsed))
```

### Python - 可变帧率控制

```python
# 不需要固定循环时间
# 底层自动以 50Hz 维持控制

while running:
    # 可能耗时的处理 (如视觉处理)
    image = capture_image()
    target = process_vision(image)  # 可能 10-100ms
    
    # 无论何时准备好都可以发送
    controller.set_joint_position_target(target)
    
    # 底层控制器继续以 50Hz 运行
```

### C++ 使用

```cpp
#include "franka_async_controller.h"

using namespace franka_control;

int main() {
    // 创建控制器
    ControllerConfig config;
    config.verbose = true;
    
    FrankaAsyncController controller("172.16.0.2", config);
    
    // 启动
    controller.start();
    
    // 发送目标
    std::array<double, 7> target = {0, -M_PI_4, 0, -3*M_PI_4, 0, M_PI_2, M_PI_4};
    controller.setJointPositionTarget(target);
    
    // 获取状态
    auto state = controller.getRobotState();
    if (state.has_value()) {
        std::cout << "Position: " << state->q[0] << std::endl;
    }
    
    // 停止
    controller.stop();
    
    return 0;
}
```

## API 参考

### Python API

#### ControllerConfig

```python
config = fc.ControllerConfig()
config.maximum_joint_velocities = [0.655, 0.655, 0.655, 0.655, 1.315, 1.315, 1.315]
config.goal_tolerance = 10.0
config.control_period_ms = 20  # 50Hz
config.verbose = False
```

#### FrankaAsyncController

**方法：**

- `start() -> bool`: 启动控制循环
- `stop()`: 停止控制循环
- `set_joint_position_target(positions: np.ndarray) -> bool`: 设置目标位置
- `get_robot_state() -> RobotState | None`: 获取当前状态
- `get_latest_target() -> JointPositionCommand | None`: 获取最新目标
- `is_running() -> bool`: 检查是否运行
- `has_error() -> bool`: 检查是否有错误
- `get_error_message() -> str`: 获取错误信息
- `get_control_frequency() -> float`: 获取实测控制频率

#### RobotState

```python
state = controller.get_robot_state()
print(state.q)        # 关节位置 (7,)
print(state.dq)       # 关节速度 (7,)
print(state.tau_j)    # 关节力矩 (7,)
print(state.O_T_EE)   # 末端位姿 (4, 4)
print(state.timestamp)
```

## 测试程序

### 1. C++ 测试

```bash
./build/test_controller 172.16.0.2
```

### 2. Python 固定帧率测试

```bash
python3 test_controller_fixed_rate.py 172.16.0.2
```

特性：
- 10Hz 目标更新
- 底层 50Hz 控制
- 每秒切换运动方向

### 3. Python 可变帧率测试

```bash
python3 test_controller_variable_rate.py 172.16.0.2
```

特性：
- 在 5-30Hz 之间动态切换
- 展示不同帧率下的表现
- 变化的运动幅度

### 4. 高级视觉控制示例

```bash
python3 advanced_vision_control.py 172.16.0.2
```

特性：
- 模拟视觉处理 (10-100ms 可变延迟)
- 异步目标生成
- 实时性能统计
- 展示实际应用场景

## 性能特点

- **底层控制频率**：固定 50Hz (由 `AsyncPositionControlHandler` 保证)
- **Python 调用频率**：无限制，根据应用需求
- **延迟**：
  - Python → C++：< 1ms (pybind11)
  - C++ 缓冲更新：< 0.1ms (mutex)
  - 控制循环：20ms ± 1ms

## 常见问题

### Q: Python 层应该以什么频率发送目标？

A: 取决于应用。底层以 50Hz 运行，所以：
- 如果目标更新 < 50Hz：每个目标都会被使用
- 如果目标更新 > 50Hz：只使用最新的目标
- 推荐：10-30Hz，给处理留出余量

### Q: 如何处理视觉处理延迟？

A: 这正是此架构的优势：
- 视觉处理可以慢 (50-100ms)
- 底层控制器继续以 50Hz 运行
- 一旦有新目标就更新，无需等待

### Q: 线程安全吗？

A: 是的，所有共享数据都有 mutex 保护：
- `setJointPositionTarget()` 可以从任何线程调用
- `getRobotState()` 可以从任何线程调用
- 内部控制循环独立运行

### Q: 如何调整控制参数？

A: 修改 `ControllerConfig`：
```python
config = fc.ControllerConfig()
config.maximum_joint_velocities = [...]  # 降低速度
config.goal_tolerance = 5.0  # 更严格的到达判断
config.control_period_ms = 10  # 更快的控制 (100Hz)
```

## 扩展建议

1. **添加笛卡尔空间控制**：扩展支持末端位姿目标
2. **轨迹队列**：支持多个目标点的队列
3. **力控制**：集成力矩控制模式
4. **状态估计**：添加滤波器平滑状态输出
5. **安全检查**：添加工作空间限制、速度限制等

## 许可证

基于 Franka Robotics 的 Apache-2.0 许可证

## 参考

- [libfranka Documentation](https://frankaemika.github.io/libfranka/)
- [pybind11 Documentation](https://pybind11.readthedocs.io/)
