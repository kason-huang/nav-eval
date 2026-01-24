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
│                        VLNEvaluator                             │
│  ┌──────────────────┐    ┌──────────────────┐                   │
│  │ Dataset Loader   │    │  Result Manager  │                   │
│  │ - R2R format     │    │ - JSON output    │                   │
│  └──────────────────┘    └──────────────────┘                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Evaluation Loop                             │   │
│  │  for episode in dataset:                                 │   │
│  │    obs = env.reset(episode)                             │   │
│  │    obs['instruction'] = episode.instruction              │   │
│  │    while not done:                                       │   │
│  │      action = policy.act(obs)                            │   │
│  │      obs, reward, done, info = env.step(action)          │   │
│  │      obs['instruction'] = episode.instruction            │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
         │                              │
         ▼                              ▼
┌─────────────────┐           ┌─────────────────┐
│      Env        │           │     Policy      │
│  (Environment   │           │  (Action        │
│   Interface)    │           │   Selection)    │
│  ┌────────────┐ │           │  ┌────────────┐ │
│  │ MockEnv    │ │           │  │ MockPolicy │ │
│  │ O3DEEnv    │ │           │  │ WebSocket  │ │
│  │   └──O3DE  │ │           │  │   Policy   │ │
│  │   Simulator│ │           │  └────────────┘ │
│  └────────────┘ │           └─────────────────┘
└─────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────┐
│                         O3DE Simulator                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   场景      │  │   机器人    │  │  ROS2 Sensor │              │
│  │  (动态加载) │  │  Entity     │  │  RGB/Depth   │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
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

#### 3.2.1 Env接口（环境接口）
遵循Habitat-lab标准的Env抽象接口，封装仿真器内部实现

**功能**：
- 环境重置（reset）：根据Episode配置初始化场景
- 步进执行（step）：执行动作并返回观察、奖励、完成标志和信息
- 资源清理（close）：释放环境资源

**接口设计**：
```python
class Env(abc.ABC):
    """Environment interface following Habitat-lab conventions"""

    @abc.abstractmethod
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
        """Reset environment with episode config

        Returns:
            obs: {'rgb': ndarray, 'depth': ndarray}
        """
        pass

    @abc.abstractmethod
    def step(self, action: int) -> Tuple[obs, reward, done, info]:
        """Execute one step

        Returns:
            obs: {'rgb': ndarray, 'depth': ndarray}
            reward: float (0.0 for VLN)
            done: bool
            info: {'position', 'distance_to_goal', 'collision_count', 'trajectory'}
        """
        pass

    @abc.abstractmethod
    def close(self):
        """Clean up environment resources"""
        pass
```

**实现类**：
- `MockEnv`：Mock环境，用于测试（无O3DE/ROS2依赖）
- `O3DEEnv`：生产环境，内部封装O3DESimulator

#### 3.2.2 Policy接口（策略接口）
动作选择的抽象接口

**功能**：
- 根据观察选择动作
- 支持本地和远程VLM模型

**接口设计**：
```python
class Policy(abc.ABC):
    """Action selection interface"""

    @abc.abstractmethod
    def act(self, obs: Dict[str, Any]) -> int:
        """Get action from observation

        Args:
            obs: {'rgb': ndarray, 'depth': ndarray, 'instruction': str}

        Returns:
            action: 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT
        """
        pass
```

**实现类**：
- `MockPolicy`：随机策略，用于测试
- `WebSocketPolicy`：通过WebSocket连接远程VLM服务器

#### 3.2.3 VLNEvaluator主模块
评测流程编排，使用Env和Policy接口

**功能**：
- 加载R2R数据集
- 编排评测流程（通过env.reset和policy.act）
- 记录评测数据
- 生成评测报告

**接口设计**：
```python
class VLNEvaluator:
    def __init__(self, env: Env, policy: Policy,
                 default_max_steps: int = 50,
                 default_success_threshold: float = 0.2)

    def evaluate_dataset(self, dataset_path: str) -> EvaluationResult
    def evaluate_episode(self, episode: R2REpisode) -> EpisodeResult
    def save_results(self, results: EvaluationResult, output_path: str)
    def close(self)
```

**使用上下文管理器**：
```python
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset(dataset_path)
    evaluator.save_results(results, output_path)
```

#### 3.2.4 DatasetLoader模块
负责数据集加载

**功能**：
- 加载R2R格式JSON数据集
- 提供R2REpisode数据类

**接口设计**：
```python
@dataclass
class R2REpisode:
    episode_id: str
    scene_id: str
    instruction: str
    start_position: Dict[str, float]
    start_rotation: Dict[str, float]
    goal_position: Dict[str, float]
    scene_config_path: Optional[str] = None
    max_steps: Optional[int] = None
    success_threshold: Optional[float] = None

class R2RDatasetLoader:
    @staticmethod
    def load_from_file(dataset_path: str) -> List[R2REpisode]
```

#### 3.2.5 O3DESimulator模块
（内部实现，被O3DEEnv封装）

**功能**：
- O3DE场景和机器人管理
- ROS2传感器订阅
- 动作执行
- 位姿查询

**注意**：O3DESimulator不直接暴露给VLNEvaluator，而是通过O3DEEnv接口访问

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
初始化Env和Policy
  ├─ env = O3DEEnv(simulator_config) 或 MockEnv()
  └─ policy = WebSocketPolicy(...) 或 MockPolicy()
  │
  ▼
┌─────────────────────────┐
│  for each episode:      │
│                         │
│  1. env.reset(episode)  │
│  2. obs['instruction']  │
│     = episode.instruction│
│  3. 评测循环：          │
│     while not done:     │
│       a. action =       │
│          policy.act(obs)│
│       b. obs, reward,   │
│          done, info =   │
│          env.step(action)│
│       c. obs['instruction│
│          '] = episode.  │
│          instruction    │
│  4. 记录结果           │
└─────────────────────────┘
  │
  ▼
生成汇总统计
  │
  ▼
保存JSON结果
  │
  ▼
关闭env (env.close())
  │
  ▼
结束
```

### 5.2 详细评测循环伪代码

```python
def evaluate_episode(env: Env, policy: Policy, r2r_ep: R2REpisode,
                    default_max_steps=50, default_success_threshold=0.2):
    """
    单episode评测流程 - 使用Env和Policy接口
    """
    # 1. 转换R2REpisode为Episode
    max_steps = r2r_ep.max_steps or default_max_steps
    success_threshold = r2r_ep.success_threshold or default_success_threshold

    episode = Episode(
        episode_id=r2r_ep.episode_id,
        scene_id=r2r_ep.scene_id,
        start_position=r2r_ep.start_position,
        start_rotation=r2r_ep.start_rotation,
        goal_position=r2r_ep.goal_position,
        instruction=r2r_ep.instruction,
        scene_config_path=r2r_ep.scene_config_path,
        max_steps=max_steps,
        success_threshold=success_threshold
    )

    # 2. Reset环境
    obs = env.reset(episode)

    # 3. 添加instruction到observation (VLN需要文本指令)
    obs['instruction'] = r2r_ep.instruction

    # 4. 初始化状态
    success = False
    failure_reason = None

    # 5. 评测循环
    for step in range(max_steps):
        # 5.1 通过policy获取动作
        action = policy.act(obs)

        # 5.2 通过env执行动作
        obs, reward, done, info = env.step(action)

        # 5.3 重新添加instruction到observation (下一步使用)
        obs['instruction'] = r2r_ep.instruction

        # 5.4 检查是否完成
        if done:
            # success由distance_to_goal决定
            success = (info['distance_to_goal'] < success_threshold)
            if not success:
                failure_reason = 'timeout'
            break

    # 6. 返回结果
    return EpisodeResult(
        episode_id=r2r_ep.episode_id,
        scene_id=r2r_ep.scene_id,
        instruction=r2r_ep.instruction,
        success=success,
        failure_reason=failure_reason,
        final_distance_to_goal=info['distance_to_goal'],
        steps=step + 1,
        collision_count=info.get('collision_count', 0),
        trajectory=info.get('trajectory', [])
    )
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
│   ├── VLN_EVALUATION_DESIGN.md       # 本文档
│   ├── O3DE_SIMULATOR_DESIGN.md       # O3DE Simulator设计文档
│   └── IMPLEMENTATION_DETAILS.md      # 实现细节文档
├── vln_evaluator/                     # VLN评测器模块包
│   ├── __init__.py                    # 模块导出
│   ├── env.py                         # Env接口（Env, MockEnv, O3DEEnv）
│   ├── policy.py                      # Policy接口（Policy, MockPolicy, WebSocketPolicy）
│   ├── dataset_loader.py              # R2R数据集加载器
│   └── vln_evaluator.py               # 主评测器
├── o3de_sim/                          # O3DE仿真接口库（现有）
│   ├── o3de_api/
│   │   ├── o3de_agent.py              # O3DE Agent
│   │   ├── socket/                    # Socket模式API
│   │   ├── peb/                       # PEB模式API
│   │   └── interface/                 # 通用接口
│   └── common/
│       └── scene_builder/             # 场景构建器
├── o3de_simulator.py                  # O3DE仿真器（内部实现）
├── action_executor.py                 # ROS2动作执行器（现有，被O3DESimulator使用）
├── run_vln_eval.py                    # 评测运行脚本
├── configs/
│   └── vln_eval.yaml                  # 评测配置文件
├── datasets/
│   └── sample_r2r.json                # 示例R2R数据集
└── results/                           # 评测结果输出目录
```

### 使用示例

**示例1：使用Mock组件进行测试**
```python
from vln_evaluator import VLNEvaluator, MockEnv
from vln_evaluator.policy import MockPolicy

env = MockEnv()
policy = MockPolicy()
evaluator = VLNEvaluator(env, policy)
results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
evaluator.save_results(results, 'results/test_eval.json')
```

**示例2：使用O3DE + WebSocket VLM**
```python
from vln_evaluator import VLNEvaluator, O3DEEnv
from vln_evaluator.policy import WebSocketPolicy

env = O3DEEnv(simulator_config={'mode': 'socket'})
policy = WebSocketPolicy('localhost', '8080')
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    evaluator.save_results(results, 'results/o3de_eval.json')
```

**示例3：使用便捷函数**
```python
from vln_evaluator import evaluate_vln
from vln_evaluator.env import O3DEEnv
from vln_evaluator.policy import WebSocketPolicy

env = O3DEEnv(simulator_config={'mode': 'socket'})
policy = WebSocketPolicy('192.168.1.100', '9999')
evaluate_vln(
    dataset_path='datasets/sample_r2r.json',
    output_path='results/eval.json',
    env=env,
    policy=policy
)
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

### 架构变更

系统已重构为使用Env和Policy接口，主要变更：

1. **VLNEvaluator现在使用`env`和`policy`接口**，不再直接访问simulator_config和vlm_config
2. **Env接口** (`Env`, `MockEnv`, `O3DEEnv`) 将仿真器内部封装，遵循Habitat-lab约定
3. **Policy接口** (`Policy`, `MockPolicy`, `WebSocketPolicy`) 处理动作选择
4. **Observation字典**包含 `{'rgb', 'depth', 'instruction'}`，其中instruction由VLNEvaluator注入

### 已实现模块

#### 1. vln_evaluator/env.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/env.py`

**实现内容**:
- `Episode` 数据类：Episode配置
- `Env` 抽象基类：环境接口（reset/step/close）
- `MockEnv`：Mock环境，用于测试（无O3DE/ROS2依赖）
- `O3DEEnv`：生产环境，内部封装O3DESimulator

**核心接口**:
```python
# 抽象接口
class Env(abc.ABC):
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]
    def step(self, action: int) -> Tuple[obs, reward, done, info]
    def close(self)

# 使用示例
env = O3DEEnv(simulator_config={'mode': 'socket'})
obs = env.reset(episode)  # {'rgb': ..., 'depth': ...}
obs, reward, done, info = env.step(action)
env.close()
```

#### 2. vln_evaluator/policy.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/policy.py`

**实现内容**:
- `Policy` 抽象基类：动作选择接口
- `MockPolicy`：随机策略，用于测试
- `WebSocketPolicy`：通过WebSocket连接远程VLM服务器

**核心接口**:
```python
class Policy(abc.ABC):
    def act(self, obs: Dict[str, Any]) -> int  # 返回 0/1/2/3

# 使用示例
policy = WebSocketPolicy('localhost', '8080')
action = policy.act(obs)  # obs包含 {'rgb', 'depth', 'instruction'}
```

#### 3. vln_evaluator/vln_evaluator.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/vln_evaluator.py`

**实现内容**:
- `EpisodeResult` 数据类：单episode结果
- `EvaluationResult` 数据类：整体评测结果
- `VLNEvaluator` 类：主评测器，使用Env和Policy接口

**核心接口**:
```python
# 使用Env和Policy接口初始化
evaluator = VLNEvaluator(env=env, policy=policy,
                         default_max_steps=50,
                         default_success_threshold=0.2)

# 评测数据集
results = evaluator.evaluate_dataset(dataset_path)

# 保存结果
evaluator.save_results(results, output_path)

# 使用上下文管理器
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset(dataset_path)
    evaluator.save_results(results, output_path)
```

#### 4. vln_evaluator/dataset_loader.py
**位置**: `/root/workspace/cloudrobo/nav-eval/vln_evaluator/dataset_loader.py`

**实现内容**:
- `R2REpisode` 数据类
- `R2RDatasetLoader` 类：加载/保存R2R格式数据集
- `create_sample_dataset()` 函数：创建示例数据集

#### 5. o3de_simulator.py (内部实现)
**位置**: `/root/workspace/cloudrobo/nav-eval/o3de_simulator.py`

**实现内容**:
- `O3DESimulator` 类：Habitat风格的仿真器接口（reset/step/close）
- `SensorSubscriber`：ROS2传感器订阅器（RGB/Depth/Scan）
- `ActionExecutorWrapper`：动作执行器封装（复用action_executor逻辑）
- `O3DEAgentManager`：O3DE Agent管理器

**注意**: O3DESimulator不直接暴露给VLNEvaluator，而是通过O3DEEnv接口访问

#### 6. run_vln_eval.py
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
│   ├── O3DE_SIMULATOR_DESIGN.md       # O3DE Simulator设计文档
│   └── IMPLEMENTATION_DETAILS.md      # 实现细节文档
├── vln_evaluator/                     # VLN评测器模块
│   ├── __init__.py                    # 模块导出
│   ├── env.py                         # Env接口
│   ├── policy.py                      # Policy接口
│   ├── dataset_loader.py              # R2R数据集加载器
│   └── vln_evaluator.py               # 主评测器
├── o3de_simulator.py                  # O3DE仿真器（内部实现）
├── action_executor.py                 # ROS2动作执行器（被O3DESimulator使用）
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
