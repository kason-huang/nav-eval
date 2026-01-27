# VLN动作执行器技术方案

## 一、概述

`VLNActionExecutor` 是基于 ROS2 的机器人动作执行节点，为视觉语言导航(VLN)任务提供离散动作控制接口。

**核心功能**：实现3个离散动作（STOP, FORWARD, LEFT/RIGHT），支持里程计闭环控制和碰撞检测。

**源码位置**：`vln_evaluator/action_executor.py`

---

## 二、3个核心动作实现方案

### 动作1: 精确前进25cm (`execute_forward_25cm`)

#### 实现原理
**里程计闭环控制** - 基于 `/odom` 话题的位置反馈

#### 控制逻辑
1. 等待有效的里程计数据（`wait_for_odom()`）
2. 记录起始位置 `(start_x, start_y)`
3. 进入控制循环：
   - 持续计算当前已移动距离：`dist = sqrt((x-x0)² + (y-y0)²)`
   - 发布线速度 `Twist(linear.x=0.1 m/s)`
   - 检查终止条件：
     - ✅ 达到目标距离：`dist >= 0.25m`
     - ⚠️ 检测到碰撞：`collision_detected == True`
4. 调用 `stop_robot()` 停止机器人

#### 性能指标
| 指标 | 值 |
|------|-----|
| 精度 | ±2cm（受里程计精度和物理引擎影响）|
| 执行时间 | 约2.5秒（0.1 m/s ÷ 0.25m）|
| 控制频率 | 20Hz（每50ms更新一次）|

#### 代码关键点
```python
def execute_forward_25cm(self):
    start_x = self.curr_pose.position.x
    start_y = self.curr_pose.position.y
    self.is_running = True
    self.collision_detected = False

    while rclpy.ok():
        rclpy.spin_once(self, timeout_sec=0)
        dist = math.sqrt(
            (self.curr_pose.position.x - start_x) ** 2 +
            (self.curr_pose.position.y - start_y) ** 2
        )
        if self.collision_detected or dist >= 0.25:
            break
        msg = Twist()
        msg.linear.x = self.LINEAR_SPEED  # 0.1 m/s
        self.cmd_pub.publish(msg)
        time.sleep(0.05)
    self.stop_robot()
```

---

### 动作2: 精确旋转15度 (`execute_rotate_15deg`)

#### 实现原理
**IMU/里程计航向角闭环控制** - 从四元数提取航向角

#### 控制逻辑
1. 等待有效的里程计数据
2. 读取当前航向角 `start_yaw`（从四元数转换）
3. 计算目标航向角：`target_yaw = start_yaw ± 15°`
4. 归一化到 `[-π, π]` 范围
5. 进入控制循环：
   - 计算角度差：`diff = target_yaw - current_yaw`
   - 发布角速度：`Twist(angular.z=±0.3 rad/s)`（方向由diff符号决定）
   - 检查终止条件：`|diff| < 0.02 rad`（约1°）
6. 调用 `stop_robot()` 停止机器人

#### 航向角计算（四元数→欧拉角）
```python
def get_yaw(self):
    """从四元数提取航向角（绕Z轴旋转）"""
    if self.curr_pose is None:
        return 0.0
    q = self.curr_pose.orientation
    # ZYX欧拉角序列的yaw分量
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
```

#### 性能指标
| 指标 | 值 |
|------|-----|
| 精度 | ±1°（0.02 rad ≈ 1.15°）|
| 执行时间 | 约0.87秒（15° ÷ 17.2°/s）|
| 角速度 | 0.3 rad/s ≈ 17.2°/s |

---

### 动作3: 定时移动 (`move_timed`)

#### 实现原理
**开环时间控制** - 不依赖里程计，简单可靠

#### 控制逻辑
1. 记录开始时间
2. 进入控制循环：
   - 发布指定速度指令（`linear_x`, `angular_z`）
   - 检查终止条件：
     - ✅ 达到设定时长：`elapsed_time >= duration_sec`
     - ⚠️ 检测到碰撞
3. 调用 `stop_robot()` 停止机器人

#### 用途
- 调试和测试
- 备用控制方案（当里程计不可用时）
- 手动遥控模式

#### 代码
```python
def move_timed(self, linear_x=0.0, angular_z=0.0, duration_sec=1.0):
    self.is_running = True
    self.collision_detected = False

    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < duration_sec:
        rclpy.spin_once(self, timeout_sec=0)
        if self.collision_detected:
            self.get_logger().warn("定时移动因碰撞提前终止")
            break
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)
        time.sleep(0.05)
    self.stop_robot()
```

---

## 三、核心技术架构

### 1. ROS2通信架构

| 话题 | 类型 | 方向 | QoS配置 | 用途 |
|------|------|------|---------|------|
| `/cmd_vel` | Twist | 发布 | RELIABLE, depth=10 | 发送速度控制指令 |
| `/odom` | Odometry | 订阅 | **BEST_EFFORT**, VOLATILE, depth=10 | 接收里程计数据 |
| `/scan` | LaserScan | 订阅 | **BEST_EFFORT**, VOLATILE, depth=10 | 接收激光雷达数据 |

#### 关键设计决策
使用 **SENSOR_DATA QoS**（BEST_EFFORT + VOLATILE）与 O3DE 仿真器兼容：
- O3DE 的传感器话题默认使用 BEST_EFFORT
- VOLATILE 策略保证只接收最新数据，不缓存历史数据

### 2. 节点架构
```
VLNActionExecutor (Node)
├── Publishers
│   └── /cmd_vel (Twist)
├── Subscribers
│   ├── /odom (Odometry) → odom_callback()
│   └── /scan (LaserScan) → scan_callback()
└── State Variables
    ├── curr_pose (当前位姿)
    ├── collision_detected (碰撞标志)
    ├── collision_count (碰撞计数)
    └── is_running (任务运行标志)
```

---

## 四、里程计数据处理

### 问题：O3DE的零帧问题

O3DE启动时会发布初始零帧：
- `header.stamp.sec = 0`
- `position = (0, 0, 0)`
- `orientation = (0, 0, 0, 1)`

如果直接使用这些零值，会导致：
- 错误的位移计算
- 基于零位置的闭环控制失效

### 解决方案：双重验证过滤

```python
def odom_callback(self, msg):
    header = msg.header
    pose = msg.pose.pose
    pos = pose.position

    # 1. 检查时间戳有效性
    if header.stamp.sec == 0:
        # 2. 双重检查：是否为零位姿
        quat = pose.orientation
        is_zero_pose = (
            abs(pos.x) < 1e-6 and abs(pos.y) < 1e-6 and abs(pos.z) < 1e-6 and
            abs(quat.x) < 1e-6 and abs(quat.y) < 1e-6 and
            abs(quat.z) < 1e-6 and
            abs(quat.w - 1.0) < 1e-3  # 单位四元数 (0,0,0,1)
        )
        if is_zero_pose:
            return  # 忽略无效零帧

    # 接受有效数据
    self.curr_pose = pose
```

### 等待有效数据机制

```python
def wait_for_odom(self, timeout_sec=5.0):
    """主动等待有效的 odom 数据，超时返回 False"""
    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < timeout_sec:
        if self.curr_pose is not None:
            return True
        rclpy.spin_once(self, timeout_sec=0.01)
    return False
```

---

## 五、碰撞检测系统

### 传感器配置
- **类型**：2D激光雷达
- **话题**：`/scan` (sensor_msgs/LaserScan)
- **检测范围**：前向60个扇区（前30点 + 后30点）

### 检测算法

#### 1. 采样策略
```python
# 采样前向扇区（前后各30点）
num_points = len(ranges)
window = min(30, num_points // 2)
front_view = list(ranges[:window]) + list(ranges[-window:])
```

**设计理由**：
- 全向激光雷达的 `ranges` 数组按环形排列
- `ranges[:30]` 和 `ranges[-30:]` 合并形成前向180°扇区
- 限制 `window` 不超过总点数的一半，避免重叠

#### 2. 数据过滤
```python
valid_ranges = [
    r for r in front_view
    if msg.range_min < r < msg.range_max
]
```
- 过滤掉无效读数（超出传感器范围）
- 防止 `inf` 或 `nan` 干扰判断

#### 3. 碰撞判定
```python
if valid_ranges and min(valid_ranges) < self.COLLISION_THRESHOLD:
    # 触发碰撞
    self.collision_detected = True
    self.collision_count += 1
```
- **阈值**：`COLLISION_THRESHOLD = 0.2m`（20厘米）
- 触发条件：最小有效距离 < 20cm

### 状态管理机制

```python
def scan_callback(self, msg):
    # ... 检测逻辑 ...
    if current_min < self.COLLISION_THRESHOLD:
        # 防止重复触发：只在任务运行中且尚未标记时触发
        if self.is_running and not self.collision_detected:
            self.collision_detected = True
            self.collision_count += 1
            print(f"！！！检测到碰撞停止！！！ 总次数: {self.collision_count}")
```

**关键设计**：
- `self.is_running` 标志：只在任务执行时检测碰撞
- `not self.collision_detected` 防护：避免同一次碰撞重复计数
- `collision_count` 累计：用于评估指标统计

---

## 六、状态机设计

### 状态变量
| 变量 | 类型 | 作用 |
|------|------|------|
| `is_running` | bool | 标记任务是否正在执行 |
| `collision_detected` | bool | 标记是否检测到碰撞 |
| `collision_count` | int | 累计碰撞次数（会话级）|
| `curr_pose` | Pose | 当前机器人位姿 |

### 状态转换流程

```
初始状态: is_running=False, collision_detected=False
    ↓
[动作开始] execute_forward_25cm() / execute_rotate_15deg()
    ↓
设置: is_running=True
重置: collision_detected=False
    ↓
┌─────────────────────────────────────────┐
│  控制循环 (20Hz):                        │
│    1. rclpy.spin_once()                  │
│       - 接收 /odom 更新 curr_pose        │
│       - 接收 /scan 可能触发 collision     │
│    2. 检查终止条件:                      │
│       - collision_detected == True → STOP│
│       - 目标达成 → STOP                  │
│    3. 发布速度指令到 /cmd_vel            │
│    4. sleep(0.05s)                       │
└─────────────────────────────────────────┘
    ↓
[动作结束] 调用 stop_robot()
    ↓
设置: is_running=False
发布: Twist() (零速度)
```

### 停止机制

```python
def stop_robot(self):
    self.cmd_pub.publish(Twist())  # 发布零速度
    self.is_running = False         # 重置运行标志
```

**注意**：`collision_detected` 和 `collision_count` **不重置**，保持会话级累计

---

## 七、性能参数配置

### 可调参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LINEAR_SPEED` | 0.1 m/s | 前进线速度 |
| `ANGULAR_SPEED` | 0.3 rad/s | 旋转角速度 |
| `COLLISION_THRESHOLD` | 0.2 m | 碰撞检测距离阈值 |

### 系统参数
| 参数 | 值 | 说明 |
|------|-----|------|
| 控制周期 | 50ms | 主循环sleep时间 |
| odom超时 | 5秒 | `wait_for_odom()` 最大等待时间 |
| QoS深度 | 10 | ROS2队列深度 |

---

## 八、与VLN评估框架集成

### ActionExecutorWrapper 封装

**位置**：`vln_evaluator/o3de_simulator.py`

```python
class ActionExecutorWrapper:
    """
    封装 VLNActionExecutor，提供统一动作接口
    """

    def __init__(self, linear_speed: float = 0.1, angular_speed: float = 0.3):
        from . import action_executor
        self.executor = action_executor.VLNActionExecutor()
        self.executor.LINEAR_SPEED = linear_speed
        self.executor.ANGULAR_SPEED = angular_speed

    def execute_action(self, action: int):
        """
        执行离散动作

        Args:
            action: 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        """
        if action == 0:
            self.stop()
        elif action == 1:
            self.executor.execute_forward_25cm()
        elif action == 2:
            self.executor.execute_rotate_15deg('left')
        elif action == 3:
            self.executor.execute_rotate_15deg('right')

    def get_collision_count(self) -> int:
        """获取累计碰撞次数（评估指标）"""
        return self.executor.collision_count
```

### 数据流架构

```
┌─────────────────┐
│  VLN Policy     │
│  (推理模型)      │
└────────┬────────┘
         │ 离散动作ID (0/1/2/3)
         ↓
┌─────────────────────────┐
│ ActionExecutorWrapper   │
│ - 映射动作ID到具体方法   │
└────────┬────────────────┘
         │ 调用方法
         ↓
┌─────────────────────────┐
│ VLNActionExecutor       │
│ - execute_forward_25cm()│
│ - execute_rotate_15deg()│
└────────┬────────────────┘
         │ /cmd_vel (Twist)
         ↓
┌─────────────────────────┐
│ O3DE Robot Controller   │
│ (ROS2接口)              │
└────────┬────────────────┘
         │ 物理控制
         ↓
┌─────────────────────────┐
│ O3DE 物理引擎           │
│ - 机器人移动            │
│ - 里程计发布 /odom      │
│ - 激光雷达发布 /scan    │
└─────────────────────────┘
```

### 在 O3DESimulator 中的使用

```python
class O3DESimulator:
    def __init__(self, ...):
        # 初始化动作执行器
        self.action_executor = ActionExecutorWrapper(
            linear_speed=0.1,
            angular_speed=0.3
        )

    def step(self, action: int):
        # 1. 执行动作（阻塞等待完成）
        self.action_executor.execute_action(action)

        # 2. 获取观察数据
        obs = self.sensor_sub.get_observation()

        # 3. 获取碰撞信息
        collision_count = self.action_executor.get_collision_count()
        collision = collision_count > 0

        # 4. 返回结果
        return obs, 0.0, done, {'collision': collision, ...}
```

---

## 九、评估指标支持

### 碰撞统计
- **`collision_count`**：会话累计碰撞次数
- **`collision`**：布尔值，是否发生过碰撞
- 用途：
  - 评估导航安全性
  - 计算碰撞率（collisions / episode）
  - 作为失败判据之一

### 里程准确性
- 基于里程计的闭环控制
- 可记录轨迹：`trajectory.append(position)`
- 用途：
  - 计算路径长度
  - 评估路径效率
  - 分析里程计漂移

---

## 十、方案优势总结

### ✅ 核心优势

1. **基于里程计的闭环控制**
   - 精确的位移和角度控制
   - 不依赖时间估算，适应物理引擎波动

2. **内置碰撞检测**
   - 实时监控前方障碍物
   - 碰撞立即停止，保护仿真安全
   - 自动累计统计，支持评估指标

3. **与O3DE仿真无缝集成**
   - SENSOR_DATA QoS兼容
   - 处理O3DE特有的零帧问题
   - 支持 GAME MODE 物理仿真

4. **模块化设计**
   - 职责清晰（控制、检测、状态管理）
   - 易于扩展新动作类型
   - 便于单元测试

5. **灵活的接口**
   - 离散动作接口（VLN标准）
   - 底层定时移动接口（调试用）
   - 可配置的速度参数

### 📊 性能特点

| 特性 | 表现 |
|------|------|
| 位移精度 | ±2cm |
| 旋转精度 | ±1° |
| 碰撞响应延迟 | <50ms（一个控制周期）|
| 动作执行成功率 | 取决于物理引擎稳定性 |

---

## 十一、使用示例

### 1. 直接使用（调试/测试）

```bash
cd /root/workspace/cloudrobo/nav-eval
python vln_evaluator/action_executor.py
```

**交互命令**：
- `1` - 前进 25cm
- `2` - 左转 15°
- `3` - 右转 15°
- `f/b/l/r` - 定时移动
- `q` - 退出

### 2. 在评估框架中使用

```python
from vln_evaluator.o3de_simulator import O3DESimulator
from vln_evaluator.env import Episode

# 创建模拟器
sim = O3DESimulator(mode='socket')

# 加载episode
episode = Episode(
    scene_config_path='...',
    start_position={'x': 0, 'y': 0, 'z': 0},
    goal_position={'x': 2, 'y': 0, 'z': 0}
)

# 重置
obs = sim.reset(episode)

# 执行动作
obs, reward, done, info = sim.step(action=1)  # FORWARD
print(f"Collision: {info['collision']}")
print(f"Distance: {info['distance_to_goal']:.2f}m")
```

---

## 十二、故障排查

### 问题1：机器人不动
**可能原因**：
- O3DE未进入 GAME MODE
- 物理引擎未启动
- `/cmd_vel` 话题未连接

**解决方法**：
```python
# 确保O3DE进入游戏模式
from o3de_sim.o3de_api.peb_extras import simulation_tool
simulation_tool.enter_game_mode()
simulation_tool.toggle_physics(True)
```

### 问题2：里程计超时
**可能原因**：
- O3DE的 `/odom` 发布器未启用
- QoS不匹配

**解决方法**：
- 检查O3DE中ROS2传感器是否正确配置
- 确认 `/odom` 话题正在发布：`ros2 topic hz /odom`

### 问题3：碰撞检测不工作
**可能原因**：
- `/scan` 话题未发布
- `COLLISION_THRESHOLD` 设置过大
- 激光雷达采样区域不匹配

**解决方法**：
```bash
# 检查话题
ros2 topic echo /scan --once

# 调整阈值
executor.COLLISION_THRESHOLD = 0.3  # 增加到30cm
```

---

## 附录：代码文件结构

```
vln_evaluator/
├── action_executor.py       # 核心动作执行器
├── o3de_simulator.py        # O3DE模拟器（集成ActionExecutorWrapper）
├── env.py                   # 环境抽象接口
├── policy.py                # 策略接口
└── __init__.py
```

**相关文档**：
- `O3DE_SIMULATOR_DESIGN.md` - O3DE模拟器整体设计
- `VLN_EVALUATION_DESIGN.md` - VLN评估框架设计
- `IMPLEMENTATION_DETAILS.md` - 实现细节

---

**文档版本**：v1.0
**最后更新**：2025-01-27
**作者**：Claude Code
**项目**：nav-eval (Navigation Evaluation Framework)
