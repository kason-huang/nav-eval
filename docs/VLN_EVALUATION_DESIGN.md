# O3DE VLN评测系统 - 需求分析与设计文档

## 一、需求概述

在O3DE仿真环境中构建一个视觉-语言导航(VLN)评测系统，模型根据当前的RGB图像、深度图像和文本描述指令，输出导航动作，评测其导航能力。

## 二、系统需求

### 2.1 核心功能需求

| 需求项 | 说明 |
|--------|------|
| **仿真环境** | 纯O3DE，完全使用O3DE作为仿真环境 |
| **模型部署** | VLM模型通过WebSocket远程服务部署 |
| **传感器输入** | RGB图像、深度图像、里程计、激光扫描 |
| **动作输出** | 4个离散动作：停止(0)、前进25cm(1)、左转15°(2)、右转15°(3) |
| **动作执行** | 复用action_executor.py，通过ROS2 /cmd_vel话题控制 |
| **位姿获取** | Evaluator通过o3de_sim API直接查询机器人entity的transform |
| **Episode管理** | 从R2R格式数据集文件加载，通过o3de_sim API动态管理场景 |
| **成功判定** | STOP动作 + 距离阈值 + 固定步数超时限制 |
| **评测指标** | 成功率(SR) + 距离误差 + 碰撞次数记录 |
| **输出格式** | JSON文件记录详细结果和汇总统计 |
| **并发模式** | 单进程顺序执行 |

### 2.2 详细功能说明

#### 动作空间定义
```
0 - STOP    : 停止，表示模型认为已到达目的地
1 - FORWARD : 前进 25cm
2 - LEFT    : 左转 15°
3 - RIGHT   : 右转 15°
```

#### 传感器数据获取方式
通过ROS2话题订阅获取传感器数据：
- `/rgb`   - RGB图像数据
- `/depth` - 深度图像数据
- `/odom`  - 里程计数据（位置+朝向）
- `/scan`  - 激光扫描数据（用于碰撞检测）

#### 成功判定条件
Episode成功的条件需**同时满足**：
1. 模型输出STOP动作(0)
2. 机器人与目标点的欧氏距离 < 设定阈值（如0.2米）
3. 步数未超过最大步数限制（固定值，如50步）

位姿通过o3de_sim API直接查询机器人entity获取：
```python
current_pos = o3de_api.tc.get_position(robot_entity_id)
distance = euclidean_distance(current_pos, goal_position)
```

## 三、系统架构设计

### 3.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                         O3DE Simulator                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   场景      │  │   机器人    │  │  ROS2 Sensor │              │
│  │  (动态加载) │  │  Entity     │  │  RGB/Depth   │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
│         ▲                ▲                   │                   │
│         │                │                   │ ROS2 Topics       │
│         │ o3de_sim API   │                   ▼                   │
└─────────┼────────────────┼──────────┐ ┌─────────────┐           │
          │                │          │ │ Action       │           │
          │                │          │ │ Executor     │           │
          │                │          │ │ (复用action_ │           │
          │                │          │ │  executor.py)│           │
          │                │          └─────────────┘           │
          │                │                    │                 │
          ▼                ▼                    ▼                 │
┌─────────────────────────────────────────────────────────────────┤
│                        VLN Evaluator                             │
│  ┌──────────────────┐    ┌──────────────────┐                   │
│  │ Episode Manager  │    │  Success Judge   │                   │
│  │ - 加载R2R数据集  │    │ - O3DE API查询   │                   │
│  │ - 动态创建场景   │    │   机器人位姿     │                   │
│  │ - 设置初始位置   │    │ - 计算距离误差   │                   │
│  └──────────────────┘    └──────────────────┘                   │
│  ┌──────────────────┐    ┌──────────────────┐                   │
│  │  Sensor Subscriber│   │  Action Executor │                   │
│  │ - ROS2订阅       │    │ - ROS2 /cmd_vel  │                   │
│  │ - 缓存最新数据   │    │ - 动作执行器     │                   │
│  └──────────────────┘    └──────────────────┘                   │
│  ┌──────────────────────────────────────────┐                   │
│  │       WebSocket Policy Client            │                   │
│  │  act(rgb, depth, instruction) → 0/1/2/3  │                   │
│  └──────────────────────────────────────────┘                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ WebSocket
┌─────────────────────────────────────────────────────────────────┐
│                    Remote VLM Server                            │
│         输入: RGB图像 + Depth图像 + 文本指令                     │
│         输出: 动作 0/1/2/3                                       │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 模块设计

#### 3.2.1 EpisodeManager模块
负责数据集加载和场景管理

**功能**：
- 加载R2R格式JSON数据集
- 通过o3de_sim API动态创建场景
- 设置机器人初始位置和朝向
- 管理episode生命周期

**接口设计**：
```python
class EpisodeManager:
    def load_dataset(self, dataset_path: str) -> List[Episode]
    def setup_scene(self, scene_id: str) -> None
    def setup_robot(self, start_pos, start_rot) -> EntityId
    def get_goal_position(self) -> Vector3
    def reset_scene(self) -> None
```

#### 3.2.2 SensorSubscriber模块
负责ROS2传感器数据订阅

**功能**：
- 订阅 `/rgb`, `/depth` 图像话题
- 订阅 `/odom` 里程计话题
- 订阅 `/scan` 激光话题（碰撞检测）
- 缓存最新传感器数据

**接口设计**：
```python
class SensorSubscriber:
    def get_rgb(self) -> np.ndarray
    def get_depth(self) -> np.ndarray
    def get_odom(self) -> Pose
    def get_collision_status(self) -> bool
    def wait_for_data(self) -> None
```

#### 3.2.3 SuccessJudge模块
负责成功判定和评测指标计算

**功能**：
- 通过o3de_sim API查询机器人实时位姿
- 计算到目标的欧氏距离
- 判断episode成功/失败
- 记录评测指标

**接口设计**：
```python
class SuccessJudge:
    def get_robot_position(self) -> Vector3  # O3DE API
    def compute_distance_to_goal(self) -> float
    def is_success(self, action: int, distance: float) -> bool
    def is_timeout(self, step: int) -> bool
```

#### 3.2.4 ActionExecutor模块
复用现有action_executor.py

**功能**：
- 通过ROS2 /cmd_vel控制机器人
- 执行4种基本动作
- 碰撞检测记录

**接口设计**：
```python
class ActionExecutor:
    def execute_forward_25cm(self) -> None
    def execute_rotate_15deg(self, direction: str) -> None
    def stop_robot(self) -> None
    def get_collision_count(self) -> int
```

#### 3.2.5 WebSocketPolicyClient模块
负责与远程VLM模型通信

**功能**：
- 建立WebSocket连接
- 发送RGB、Depth、指令给模型
- 接收模型输出的动作

**接口设计**：
```python
class WebSocketPolicyClient:
    def connect(self, url: str) -> None
    def act(self, rgb: np.ndarray, depth: np.ndarray,
            instruction: str) -> int
    def disconnect(self) -> None
```

#### 3.2.6 VLNEvaluator主模块
评测流程编排

**功能**：
- 编排评测流程
- 记录评测数据
- 生成评测报告

**接口设计**：
```python
class VLNEvaluator:
    def __init__(self, config_path: str)
    def run_evaluation(self, dataset_path: str) -> EvaluationResult
    def save_results(self, output_path: str) -> None
```

## 四、数据格式定义

### 4.1 R2R数据集格式

```json
{
  "episodes": [
    {
      "episode_id": "episode_00001",
      "scene_id": "scene_001",
      "instruction": "Walk straight and turn left at the corner, then enter the room on your right",
      "start_position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
      },
      "start_rotation": {
        "x": 0,
        "y": 0,
        "z": 0
      },
      "goal_position": {
        "x": 5.0,
        "y": 0.0,
        "z": 3.0
      }
    }
  ]
}
```

**字段说明**：
- `episode_id`: 唯一标识符
- `scene_id`: 场景标识（可用于场景文件名）
- `instruction`: 导航文本指令
- `start_position`: 机器人起始位置（米）
- `start_rotation`: 机器人起始朝向（欧拉角，度）
- `goal_position`: 目标位置（米）

### 4.2 评测结果输出格式

```json
{
  "summary": {
    "total_episodes": 100,
    "success_count": 75,
    "success_rate": 0.75,
    "avg_distance_error": 0.15,
    "avg_steps": 25.3,
    "avg_collision_count": 1.2,
    "timeout_count": 5,
    "collision_failure_count": 10
  },
  "episodes": [
    {
      "episode_id": "episode_00001",
      "scene_id": "scene_001",
      "instruction": "Walk straight...",
      "success": true,
      "failure_reason": null,
      "final_distance_to_goal": 0.12,
      "steps": 18,
      "collision_count": 2,
      "trajectory": [
        {"x": 0.0, "y": 0.0, "z": 0.0},
        {"x": 0.25, "y": 0.0, "z": 0.0},
        ...
      ]
    }
  ]
}
```

**失败原因类型**：
- `timeout`: 超过最大步数
- `collision`: 碰撞（可选，当前设计只记录不终止）
- `null`: 成功

## 五、评测流程

### 5.1 主流程

```
开始
  │
  ▼
加载R2R数据集
  │
  ▼
初始化O3DE连接
  │
  ▼
初始化ROS2节点和订阅器
  │
  ▼
连接WebSocket VLM服务
  │
  ▼
┌─────────────────────────┐
│  for each episode:      │
│                         │
│  1. 加载场景            │
│  2. 设置机器人初始位置  │
│  3. 获取目标位置        │
│  4. 评测循环：          │
│     while step < max:   │
│       a. 获取RGB/Depth  │
│       b. 调用VLM获取动作│
│       c. 执行动作       │
│       d. O3DE API查询位姿│
│       e. 计算距离       │
│       f. 判断成功/失败  │
│  5. 记录结果           │
└─────────────────────────┘
  │
  ▼
生成汇总统计
  │
  ▼
保存JSON结果
  │
  ▼
结束
```

### 5.2 详细评测循环伪代码

```python
def run_episode(episode, max_steps=50, success_threshold=0.2):
    # 1. 设置场景和机器人
    episode_manager.setup_scene(episode.scene_id)
    robot_id = episode_manager.setup_robot(
        episode.start_position,
        episode.start_rotation
    )
    goal_pos = episode.goal_position

    # 2. 初始化状态
    trajectory = []
    collision_count = 0
    success = False
    failure_reason = None

    # 3. 评测循环
    for step in range(max_steps):
        # 3.1 获取传感器数据
        rgb = sensor_sub.get_rgb()
        depth = sensor_sub.get_depth()

        # 3.2 调用VLM模型
        action = ws_client.act(rgb, depth, episode.instruction)

        # 3.3 执行动作
        action_executor.execute_action(action)

        # 3.4 通过O3DE API查询机器人位姿
        current_pos = success_judge.get_robot_position()
        trajectory.append(current_pos)

        # 3.5 计算距离
        distance = success_judge.compute_distance_to_goal(current_pos, goal_pos)

        # 3.6 检测碰撞
        if sensor_sub.get_collision_status():
            collision_count += 1

        # 3.7 判断成功
        if action == 0 and distance < success_threshold:
            success = True
            break

    # 4. 处理超时
    if step >= max_steps - 1:
        failure_reason = "timeout"

    # 5. 返回结果
    return {
        "episode_id": episode.episode_id,
        "success": success,
        "failure_reason": failure_reason,
        "final_distance_to_goal": distance,
        "steps": step + 1,
        "collision_count": collision_count,
        "trajectory": trajectory
    }
```

## 六、技术实现要点

### 6.1 O3DE API位姿查询

使用o3de_sim的transform_component查询机器人实时位姿：

```python
from o3de_sim.o3de_api import o3de_agent
from o3de_sim.o3de_api.interface import math as o3de_math

agent = o3de_agent.O3DEAgent(mode=constants.O3DE_AGENT_PEB_MODE)
robot_id = agent.entity_tool.get_entity_id_by_name("robot")

# 获取位置
position = agent.tc.get_position(robot_id)  # Vector3(x, y, z)

# 获取旋转（四元数）
rotation = agent.tc.get_rotation(robot_id)  # Quaternion(w, x, y, z)
```

### 6.2 ROS2 QoS配置

O3DE的ROS2传感器使用SENSOR_DATA QoS，需要匹配配置：

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)
```

### 6.3 O3DE场景管理

使用o3de_sim的scene_builder动态管理场景：

```python
from o3de_sim.common.scene_builder import BasicSceneBuilder

# 加载场景配置
with open(scene_config_path) as f:
    scene_info = json.load(f)

# 创建场景
builder = BasicSceneBuilder(scene_info, agent)
builder.build_scene()
```

### 6.4 机器人位置设置

```python
# 设置位置
agent.tc.set_position(robot_id, o3de_math.Vector3(x, y, z))

# 设置旋转（欧拉角转四元数）
euler = o3de_math.Vector3(x_deg, y_deg, z_deg)
quat = euler_to_quaternion(euler)
agent.tc.set_rotation(robot_id, quat)
```

### 6.5 碰撞检测

复用action_executor.py中的激光扫描检测逻辑：

```python
class CollisionDetector:
    def __init__(self):
        self.collision_detected = False
        self.collision_count = 0
        self.COLLISION_THRESHOLD = 0.3  # 米

    def scan_callback(self, msg):
        ranges = msg.ranges
        mid = len(ranges) // 2
        front_view = ranges[mid-30:mid+30]

        valid_ranges = [r for r in front_view
                       if msg.range_min < r < msg.range_max]

        if valid_ranges and min(valid_ranges) < self.COLLISION_THRESHOLD:
            self.collision_detected = True
            self.collision_count += 1
```

## 七、配置参数

### 7.1 评测配置

```yaml
evaluation:
  max_steps: 50              # 最大步数
  success_threshold: 0.2     # 成功距离阈值（米）
  collision_threshold: 0.3   # 碰撞检测阈值（米）

robot:
  entity_name: "robot"       # 机器人entity名称
  linear_speed: 0.1          # 前进速度
  angular_speed: 0.3         # 旋转速度

sensors:
  rgb_topic: "/rgb"
  depth_topic: "/depth"
  odom_topic: "/odom"
  scan_topic: "/scan"

model:
  ws_url: "ws://localhost:8080"  # VLM服务地址
```

## 八、文件结构

```
nav-eval/
├── docs/
│   └── VLN_EVALUATION_DESIGN.md    # 本文档
├── o3de_sim/
│   └── (现有的o3de_sim库)
├── action_executor.py              # 现有，复用
├── policy.py                       # 现有，WebSocketPolicy
├── env.py                          # 现有，需要扩展
├── eval.py                         # 现有Habitat评测，新建O3DE评测
├── vln_evaluator/                  # 新增模块
│   ├── __init__.py
│   ├── episode_manager.py          # Episode管理
│   ├── sensor_subscriber.py        # ROS2传感器订阅
│   ├── success_judge.py            # 成功判定
│   ├── action_executor_wrapper.py  # 动作执行器封装
│   ├── ws_policy_client.py         # WebSocket客户端
│   └── vln_o3de_evaluator.py       # 主评测器
├── configs/
│   └── vln_o3de_eval.yaml         # 评测配置
├── datasets/
│   └── sample_r2r.json            # 示例数据集
└── results/
    └── (评测结果输出目录)
```

## 九、待确认事项

1. **最大步数限制**：默认50步，是否可配置？
2. **成功距离阈值**：默认0.2米，是否可配置？
3. **场景文件格式**：使用o3de_sim的scene_config.json格式？
4. **机器人模型**：是否需要在scene中预定义机器人entity？
5. **轨迹记录**：是否需要记录完整轨迹？采样频率？
6. **可视化**：是否需要生成轨迹可视化？

## 十、后续优化方向

1. **多场景并行**：支持多进程并行评测多个场景
2. **动态步数**：根据起点-目标距离动态计算最大步数
3. **更丰富的指标**：SPL、Oracle Success等
4. **可视化报告**：生成轨迹图、成功率图表等
5. **断点续评**：支持从中断的episode继续评测

---

## 十一、实现记录 (Implementation Notes)

### 已实现模块

#### 1. o3de_simulator.py
**位置**: `/root/workspace/cloudrobo/nav-eval/o3de_simulator.py`

**实现内容**:
- `O3DESimulator` 类：Habitat风格的仿真器接口（reset/step/close）
- `SensorSubscriber`：ROS2传感器订阅器（RGB/Depth/Scan）
- `ActionExecutorWrapper`：动作执行器封装（复用action_executor逻辑）
- `O3DEAgentManager`：O3DE Agent管理器
- 工具函数：欧拉角/四元数转换、距离计算等

**核心接口**:
```python
sim = O3DESimulator(mode='socket', success_threshold=0.2)
obs = sim.reset(episode)  # Returns: {'rgb': ndarray, 'depth': ndarray}
obs, reward, done, info = sim.step(action)  # info包含position, distance_to_goal, collision
sim.close()
```

#### 2. vln_evaluator/dataset_loader.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/dataset_loader.py`

**实现内容**:
- `R2REpisode` 数据类
- `R2RDatasetLoader` 类：加载/保存R2R格式数据集
- `create_sample_dataset()` 函数：创建示例数据集

#### 3. vln_evaluator/ws_policy_client.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/ws_policy_client.py`

**实现内容**:
- `WebSocketPolicyClient` 类：与远程VLM服务器通信
- 支持pickle和JSON两种序列化方式
- 支持多种响应格式（int、dict、string）
- `WebSocketPolicy` 别名类（兼容policy.py）

**核心接口**:
```python
client = WebSocketPolicyClient(url='ws://localhost:8080')
client.connect()
action = client.act(rgb, depth, instruction)
client.disconnect()
```

#### 4. vln_evaluator/vln_evaluator.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/vln_evaluator.py`

**实现内容**:
- `EpisodeResult` 数据类：单episode结果
- `EvaluationResult` 数据类：整体评测结果
- `VLNEvaluator` 类：主评测器

**核心接口**:
```python
# 方式1：使用类
with VLNEvaluator(simulator_config=..., vlm_config=...) as evaluator:
    results = evaluator.evaluate_dataset(dataset_path)
    evaluator.save_results(results, output_path)

# 方式2：使用便捷函数
evaluate_vln(dataset_path, output_path, simulator_config, vlm_config)
```

#### 5. run_vln_eval.py
**位置**: `/root/workspace/cloudrobo/nav-eval/run_vln_eval.py`

**实现内容**:
- 命令行评测脚本
- 支持YAML配置文件
- 支持命令行参数覆盖配置

### 示例文件

#### datasets/sample_r2r.json
示例R2R格式数据集，包含3个测试episode

#### configs/vln_eval.yaml
VLN评测配置文件（simulator、vlm、evaluation、logging）

### 当前文件结构

```
nav-eval/
├── docs/
│   ├── VLN_EVALUATION_DESIGN.md       # 本文档
│   └── O3DE_SIMULATOR_DESIGN.md       # O3DE Simulator设计文档
├── vln_evaluator/                     # VLN评测器模块
│   ├── __init__.py
│   ├── dataset_loader.py              # R2R数据集加载器
│   ├── ws_policy_client.py            # WebSocket VLM客户端
│   └── vln_evaluator.py               # 主评测器
├── o3de_simulator.py                  # O3DE仿真器（新增）
├── action_executor.py                 # 现有ROS2动作执行器
├── policy.py                          # 现有WebSocket Policy
├── env.py                             # 现有抽象Env类
├── run_vln_eval.py                    # 评测运行脚本
├── configs/
│   └── vln_eval.yaml                  # 评测配置文件
├── datasets/
│   └── sample_r2r.json                # 示例数据集
└── results/                           # 评测结果输出目录
```

### 待实现/优化项

1. **场景配置文件**：创建O3DE场景配置示例（scene_001.json）
2. **单元测试**：为各模块编写测试用例
3. **错误处理**：增强异常处理和错误恢复
4. **日志系统**：结构化日志记录
5. **可视化**：轨迹可视化模块

### 使用示例

```bash
# 基本使用
python run_vln_eval.py \
    --dataset datasets/sample_r2r.json \
    --output results/eval_results.json

# 指定VLM服务器
python run_vln_eval.py \
    --dataset datasets/sample_r2r.json \
    --output results/eval_results.json \
    --vlm-url ws://192.168.1.100:8080

# 评测特定episode
python run_vln_eval.py \
    --dataset datasets/sample_r2r.json \
    --output results/eval_results.json \
    --episodes ep_001 ep_002
```
