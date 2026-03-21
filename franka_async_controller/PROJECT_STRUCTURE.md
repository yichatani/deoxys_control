# 项目结构说明

## 文件组织

```
franka_async_controller/
│
├── CMakeLists.txt                      # CMake 构建配置
├── setup.py                            # Python 包安装脚本
│
├── README.md                           # 详细文档
├── QUICKSTART.md                       # 快速开始指南
│
├── franka_async_controller.h           # C++ 控制器头文件
├── franka_async_controller.cpp         # C++ 控制器实现
├── python_bindings.cpp                 # Python 绑定（pybind11）
│
├── test_controller_cpp.cpp             # C++ 测试程序
├── test_controller_fixed_rate.py       # Python 固定帧率测试
├── test_controller_variable_rate.py    # Python 可变帧率测试
├── advanced_vision_control.py          # 高级视觉控制示例
└── diffusion_policy_control.py         # Diffusion Policy 集成示例
```

## 核心文件说明

### C++ 核心 (底层实时控制)

**franka_async_controller.h**
- `FrankaAsyncController` 类定义
- `ControllerConfig`, `RobotState`, `JointPositionCommand` 结构体
- 提供完整的 C++ API

**franka_async_controller.cpp**
- 实时控制循环实现（50Hz）
- 线程安全的命令缓冲
- 与 `AsyncPositionControlHandler` 的集成

**关键设计特性：**
- 独立控制线程，保证实时性
- Mutex 保护的共享状态
- 自动错误处理和恢复
- 性能监控（频率测量）

### Python 绑定

**python_bindings.cpp**
- 使用 pybind11 将 C++ 暴露给 Python
- NumPy 数组支持
- Python 友好的异常处理

**支持的数据转换：**
- `std::array<double, 7>` ↔ `numpy.ndarray`
- `std::optional<T>` ↔ `T | None`
- C++ exceptions ↔ Python exceptions

### 测试程序

#### C++ 测试
**test_controller_cpp.cpp**
- 展示纯 C++ 使用方式
- 可变帧率控制示例
- 适合嵌入式/实时系统

#### Python 基础测试
**test_controller_fixed_rate.py**
- 10Hz 固定频率控制
- 基础功能验证
- 简单易懂的入门示例

**test_controller_variable_rate.py**
- 5-30Hz 动态变化
- 测试不同帧率下的性能
- 展示帧率灵活性

#### 高级示例
**advanced_vision_control.py**
- 模拟视觉处理管道
- 处理可变计算延迟
- 异步目标生成
- 实际应用场景参考

**diffusion_policy_control.py**
- Diffusion Policy 集成
- 动作队列管理
- 观测历史缓冲
- 完整的端到端示例

## 编译产物

编译后 `build/` 目录包含：

```
build/
├── franka_controller.cpython-38-x86_64-linux-gnu.so  # Python 模块
├── libfranka_async_controller_lib.so                 # C++ 共享库
├── test_controller                                    # C++ 可执行文件
└── CMakeFiles/                                        # CMake 临时文件
```

## 使用流程

### 开发工作流

1. **修改 C++ 代码**
   ```bash
   # 编辑 franka_async_controller.cpp 或 .h
   cd build
   make -j$(nproc)
   ```

2. **修改 Python 绑定**
   ```bash
   # 编辑 python_bindings.cpp
   cd build
   make -j$(nproc)
   ```

3. **测试**
   ```bash
   # C++
   ./test_controller 172.16.0.2
   
   # Python
   export PYTHONPATH=$PYTHONPATH:$(pwd)
   python3 ../test_controller_fixed_rate.py 172.16.0.2
   ```

### 集成到项目

**方式 1: 直接链接 C++ 库**
```cmake
# 你的 CMakeLists.txt
find_package(Franka REQUIRED)
add_executable(my_app my_app.cpp)
target_link_libraries(my_app
    franka_async_controller_lib
    Franka::Franka
)
```

**方式 2: 使用 Python 模块**
```python
# 安装
cd build
sudo make install

# 使用
import franka_controller as fc
controller = fc.FrankaAsyncController("172.16.0.2")
```

**方式 3: 作为子模块**
```bash
# 在你的项目中
git submodule add <repo-url> third_party/franka_controller

# CMakeLists.txt
add_subdirectory(third_party/franka_controller)
target_link_libraries(my_app franka_async_controller_lib)
```

## 架构层次

```
┌─────────────────────────────────────────┐
│  Application Layer                      │  ← diffusion_policy_control.py
│  (Your Control Logic)                   │    advanced_vision_control.py
├─────────────────────────────────────────┤
│  Python Wrapper                         │  ← python_bindings.cpp
│  (pybind11)                             │
├─────────────────────────────────────────┤
│  C++ Controller                         │  ← franka_async_controller.cpp
│  (Thread Management, State Buffer)     │
├─────────────────────────────────────────┤
│  AsyncPositionControlHandler            │  ← libfranka
│  (Trajectory Planning, 50Hz)           │
├─────────────────────────────────────────┤
│  FCI (Franka Control Interface)        │  ← libfranka
│  (1kHz Real-time Communication)        │
└─────────────────────────────────────────┘
```

## 数据流

### 命令路径（Python → Robot）

```
Python Application
    ↓ set_joint_position_target(np.array)
Python Bindings (pybind11)
    ↓ std::array<double, 7>
C++ Controller (mutex protected)
    ↓ latest_command_ buffer
Control Thread (50Hz)
    ↓ JointPositionTarget
AsyncPositionControlHandler
    ↓ Trajectory interpolation
Robot (1kHz)
```

### 状态路径（Robot → Python）

```
Robot (1kHz)
    ↓
AsyncPositionControlHandler
    ↓ RobotState in TargetFeedback
Control Thread (50Hz)
    ↓ Update latest_state_ buffer
C++ Controller (mutex protected)
    ↓ std::optional<RobotState>
Python Bindings
    ↓ RobotState | None
Python Application
```

## 性能考虑

### 内存布局

- **命令缓冲**: 单个 `std::array<double, 7>` (56 bytes)
- **状态缓冲**: `RobotState` (~200 bytes)
- **无动态分配**: 控制循环中零 malloc
- **缓存友好**: 连续内存访问

### 延迟分析

| 操作 | 时间 |
|------|------|
| Python → C++ 调用 | < 1 μs |
| Mutex lock/unlock | < 100 ns |
| 状态更新 | < 10 μs |
| 控制循环迭代 | 20 ms ± 1 ms |
| Python → Robot | < 40 ms |

### 线程模型

```
Main Thread (Python)
    │
    ├─→ set_joint_position_target() ──→ [Mutex] ──→ Command Buffer
    │
    └─→ get_robot_state() ──→ [Mutex] ──→ State Buffer
                                             ↑
                                             │
Control Thread (C++)                         │
    └─→ [50Hz Loop] ──→ AsyncPositionControlHandler ──→ Update
```

## 扩展指南

### 添加新的控制模式

1. 在 `franka_async_controller.h` 添加新方法
2. 在 `franka_async_controller.cpp` 实现
3. 在 `python_bindings.cpp` 暴露给 Python
4. 更新文档

示例：添加笛卡尔控制
```cpp
// .h
bool setCartesianTarget(const std::array<double, 16>& pose);

// .cpp
bool FrankaAsyncController::setCartesianTarget(...) {
    // Implementation
}

// python_bindings.cpp
.def("set_cartesian_target", ...)
```

### 添加新的传感器数据

1. 扩展 `RobotState` 结构
2. 在控制循环中更新
3. 在 Python 绑定中暴露

### 性能调优

1. **增加控制频率**: 修改 `config.control_period`
   - 注意：> 100Hz 可能需要实时内核

2. **减少延迟**: 使用 CPU 隔离
   ```cpp
   // 在控制线程中
   cpu_set_t cpuset;
   CPU_ZERO(&cpuset);
   CPU_SET(2, &cpuset);  // 使用 CPU 2
   pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
   ```

3. **优化 Python 调用**: 批量处理
   ```python
   # 不好
   for target in targets:
       controller.set_joint_position_target(target)
       time.sleep(0.01)
   
   # 好
   targets_array = np.array(targets)
   for i in range(len(targets_array)):
       controller.set_joint_position_target(targets_array[i])
   ```

## 常见陷阱

1. **忘记调用 start()**
   ```python
   controller = fc.FrankaAsyncController("172.16.0.2")
   # 忘记 controller.start()!
   controller.set_joint_position_target(...)  # 返回 False
   ```

2. **不检查错误**
   ```python
   # 总是检查
   if controller.has_error():
       print(controller.get_error_message())
   ```

3. **关节角度单位**
   - 使用弧度（radians），不是角度（degrees）
   - 检查关节限位：`q ∈ [-2.8973, 2.8973]` rad

4. **速度过快**
   ```python
   # 降低最大速度
   config.maximum_joint_velocities = [0.3] * 7  # 更安全
   ```

## 贡献

欢迎贡献！请遵循：
1. C++ 代码符合 Google C++ Style Guide
2. Python 代码符合 PEP 8
3. 添加适当的文档和测试
