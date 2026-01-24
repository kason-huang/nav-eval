# VLN评测系统实现细节文档

本文档记录VLN评测系统的实现细节、关键文件信息和核心代码片段。

## 目录

- [一、文件结构](#一文件结构)
- [二、核心模块实现](#二核心模块实现)
- [三、数据流图](#三数据流图)
- [四、关键代码片段](#四关键代码片段)
- [五、配置说明](#五配置说明)
- [六、运行流程](#六运行流程)
- [七、依赖关系](#七依赖关系)
- [八、扩展开发指南](#八扩展开发指南)

---

## 一、文件结构

### 1.1 完整目录树

```
nav-eval/
├── docs/
│   ├── VLN_EVALUATION_DESIGN.md       # VLN评测系统设计文档
│   ├── O3DE_SIMULATOR_DESIGN.md       # O3DE Simulator设计文档
│   └── IMPLEMENTATION_DETAILS.md      # 本文档（实现细节）
│
├── vln_evaluator/                     # VLN评测器模块包
│   ├── __init__.py                    # 模块导出
│   ├── env.py                         # Env接口（Env, MockEnv, O3DEEnv）
│   ├── policy.py                      # Policy接口（Policy, MockPolicy, WebSocketPolicy）
│   ├── dataset_loader.py              # R2R数据集加载器
│   └── vln_evaluator.py               # 主评测器
│
├── o3de_sim/                          # O3DE仿真接口库（现有）
│   ├── o3de_api/
│   │   ├── o3de_agent.py              # O3DE Agent
│   │   ├── socket/                    # Socket模式API
│   │   ├── peb/                       # PEB模式API
│   │   └── interface/                 # 通用接口
│   └── common/
│       └── scene_builder/             # 场景构建器
│
├── o3de_simulator.py                  # O3DE仿真器（内部实现，被O3DEEnv封装）
├── action_executor.py                 # ROS2动作执行器（被O3DESimulator使用）
├── run_vln_eval.py                    # 评测运行脚本
│
├── configs/
│   └── vln_eval.yaml                  # 评测配置文件
│
├── datasets/
│   └── sample_r2r.json                # 示例R2R数据集
│
└── results/                           # 评测结果输出目录
    └── eval_results.json              # 评测结果（运行后生成）
```

### 1.2 新增文件说明

| 文件 | 行数（约） | 主要类/函数 | 作用 |
|------|-----------|------------|------|
| `vln_evaluator/env.py` | 210 | Env, MockEnv, O3DEEnv, Episode | 环境接口 |
| `vln_evaluator/policy.py` | 60 | Policy, MockPolicy, WebSocketPolicy | 策略接口 |
| `o3de_simulator.py` | 550 | O3DESimulator, SensorSubscriber, ActionExecutorWrapper | O3DE仿真器（内部） |
| `vln_evaluator/dataset_loader.py` | 120 | R2RDatasetLoader, R2REpisode | 数据集加载 |
| `vln_evaluator/vln_evaluator.py` | 360 | VLNEvaluator, EpisodeResult, EvaluationResult | 评测流程编排 |
| `run_vln_eval.py` | 80 | main(), load_config() | 命令行入口 |
| `configs/vln_eval.yaml` | 30 | - | 配置文件 |
| `datasets/sample_r2r.json` | 80 | - | 示例数据 |

---

## 二、核心模块实现

### 2.1 vln_evaluator/env.py

#### 2.1.1 模块职责

提供Env抽象接口，遵循Habitat-lab约定。封装仿真器内部实现。

#### 2.1.2 核心类

**Episode** - Episode数据类
```python
@dataclass
class Episode:
    episode_id: str
    scene_id: str
    instruction: str
    start_position: Dict[str, float]
    start_rotation: Dict[str, float]
    goal_position: Dict[str, float]
    scene_config_path: Optional[str] = None
    max_steps: Optional[int] = None
    success_threshold: Optional[float] = None
```

**Env** - 抽象环境接口
```python
class Env(abc.ABC):
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]
    def step(self, action: int) -> Tuple[obs, reward, done, info]
    def close(self)
```

**MockEnv** - Mock环境实现
```python
class MockEnv(Env):
    def __init__(self)
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]
    def step(self, action: int) -> Tuple[obs, reward, done, info]
    def close(self)
```

**O3DEEnv** - O3DE环境实现
```python
class O3DEEnv(Env):
    def __init__(self, simulator_config: Optional[Dict[str, Any]] = None)
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]
    def step(self, action: int) -> Tuple[obs, reward, done, info]
    def close(self)
```

### 2.2 vln_evaluator/policy.py

#### 2.2.1 模块职责

提供Policy抽象接口，处理动作选择。

#### 2.2.2 核心类

**Policy** - 抽象策略接口
```python
class Policy(abc.ABC):
    def act(self, obs: Dict[str, Any]) -> int
```

**MockPolicy** - Mock策略（随机动作）
```python
class MockPolicy(Policy):
    def act(self, obs: Dict[str, Any]) -> int
```

**WebSocketPolicy** - WebSocket VLM策略
```python
class WebSocketPolicy(Policy):
    def __init__(self, ip: str, port: str)
    def act(self, obs: Dict[str, Any]) -> int
```

### 2.3 o3de_simulator.py（内部实现）

#### 2.3.1 模块职责

封装O3DE仿真环境，提供Habitat风格的Env接口。

#### 2.3.2 核心类

**O3DESimulator** - 主仿真器类
```python
class O3DESimulator:
    def __init__(self, mode, success_threshold, collision_threshold,
                 linear_speed, angular_speed)
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]
    def step(self, action: int) -> Tuple[obs, reward, done, info]
    def close(self)
```

**SensorSubscriber** - ROS2传感器订阅器
```python
class SensorSubscriber:
    def __init__(self, qos_profile: QoSProfile)
    def get_observation(self) -> Dict[str, np.ndarray]
    def reset_collision_state(self)
    def get_collision_count(self) -> int
```

**ActionExecutorWrapper** - 动作执行器
```python
class ActionExecutorWrapper:
    def __init__(self, node: Node, linear_speed, angular_speed)
    def execute_action(self, action: int)
    def execute_forward_25cm(self)
    def execute_rotate_15deg(self, direction: str)
```

**O3DEAgentManager** - O3DE Agent管理器
```python
class O3DEAgentManager:
    def __init__(self, mode: str)
    def create_scene(self, scene_config_path: str)
    def create_robot(self, name, asset_path, position, rotation)
    def set_robot_position(self, position, rotation)
    def get_robot_position(self) -> Dict[str, float]
```

#### 2.1.3 关键实现细节

1. **欧拉角转四元数** (ZYX顺序)
```python
def euler_to_quaternion(euler_deg):
    x, y, z = [math.radians(d) for d in [euler_deg['x'], euler_deg['y'], euler_deg['z']]]
    cy = math.cos(z * 0.5); sy = math.sin(z * 0.5)
    cp = math.cos(y * 0.5); sp = math.sin(y * 0.5)
    cr = math.cos(x * 0.5); sr = math.sin(x * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return o3de_math.Quaternion(qw, qx, qy, qz)
```

2. **碰撞检测逻辑** (基于LaserScan)
```python
def _scan_callback(self, msg: LaserScan):
    ranges = msg.ranges
    mid = len(ranges) // 2
    front_view = ranges[mid-30:mid+30]  # 前方60度
    valid_ranges = [r for r in front_view
                   if msg.range_min < r < msg.range_max]
    if valid_ranges and min(valid_ranges) < 0.3:  # 0.3米阈值
        self.collision_detected = True
        self.collision_count += 1
```

3. **动作执行** (基于时间开环控制)
```python
def _move_timed(self, linear_x=0.0, angular_z=0.0, duration_sec=1.0):
    start_time = time.time()
    while (time.time() - start_time) < duration_sec:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)
        rclpy.spin_once(self.node, timeout_sec=0.05)
    self.stop()
```

### 2.2 vln_evaluator/dataset_loader.py

#### 2.2.1 模块职责

加载和保存R2R格式的VLN数据集。

#### 2.2.2 核心类

**R2REpisode** - Episode数据类
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
```

**R2RDatasetLoader** - 数据集加载器
```python
class R2RDatasetLoader:
    @staticmethod
    def load_from_file(dataset_path: str) -> List[R2REpisode]

    @staticmethod
    def save_to_file(episodes: List[R2REpisode], output_path: str)
```

#### 2.2.3 支持的数据格式

```json
{
  "episodes": [
    {
      "episode_id": "ep_001",
      "scene_id": "scene_001",
      "scene_config_path": "configs/scene_001.json",  // 可选
      "instruction": "Walk straight forward for 3 meters",
      "start_position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "start_rotation": {"x": 0, "y": 0, "z": 0},  // 欧拉角，度
      "goal_position": {"x": 3.0, "y": 0.0, "z": 0.0},
      "max_steps": 50,              // 可选
      "success_threshold": 0.2       // 可选，米
    }
  ]
}
```

### 2.3 vln_evaluator/ws_policy_client.py

#### 2.3.1 模块职责

与远程VLM服务器通过WebSocket通信。

#### 2.3.2 核心类

**WebSocketPolicyClient**
```python
class WebSocketPolicyClient:
    def __init__(self, url: str, use_pickle: bool = True)
    def connect(self, url: Optional[str] = None)
    def disconnect(self)
    def act(self, rgb: np.ndarray, depth: np.ndarray,
            instruction: str) -> int
```

#### 2.3.3 通信协议

**Pickle模式** (默认，更快):
```python
# 发送
obs = {'rgb': ndarray, 'depth': ndarray, 'instruction': str}
self.ws.send(pickle.dumps(obs))

# 接收
response = pickle.loads(self.ws.recv())
action = int(response)  # 0, 1, 2, 3
```

**JSON模式** (兼容性更好):
```python
# 发送
obs = {
    'rgb': rgb.tolist(),
    'depth': depth.tolist(),
    'instruction': instruction
}
self.ws.send(json.dumps(obs))

# 接收
response = json.loads(self.ws.recv())
```

#### 2.3.4 响应格式支持

```python
def _parse_action(self, response: Any) -> int:
    if isinstance(response, int):
        return response
    elif isinstance(response, dict):
        return response.get('action', response.get('action_id', 0))
    elif isinstance(response, str):
        return {'STOP': 0, 'FORWARD': 1, 'LEFT': 2, 'RIGHT': 3}.get(response, 0)
```

### 2.4 vln_evaluator/vln_evaluator.py

#### 2.4.1 模块职责

编排整个评测流程。

#### 2.4.2 核心数据类

**EpisodeResult**
```python
@dataclass
class EpisodeResult:
    episode_id: str
    scene_id: str
    instruction: str
    success: bool
    failure_reason: Optional[str]
    final_distance_to_goal: float
    steps: int
    collision_count: int
    trajectory: List[Dict[str, float]]
```

**EvaluationResult**
```python
@dataclass
class EvaluationResult:
    total_episodes: int
    success_count: int
    success_rate: float
    avg_distance_error: float
    avg_steps: float
    avg_collision_count: float
    timeout_count: int
    episodes: List[EpisodeResult]
```

#### 2.4.3 主评测流程

```python
def evaluate_episode(self, r2r_ep: R2REpisode) -> EpisodeResult:
    """
    单episode评测流程 - 使用Env和Policy接口
    """
    # 1. 转换R2REpisode为Episode
    episode = Episode(
        episode_id=r2r_ep.episode_id,
        scene_id=r2r_ep.scene_id,
        start_position=r2r_ep.start_position,
        start_rotation=r2r_ep.start_rotation,
        goal_position=r2r_ep.goal_position,
        instruction=r2r_ep.instruction,
        max_steps=max_steps,
        success_threshold=success_threshold
    )

    # 2. Reset环境
    obs = self.env.reset(episode)

    # 3. 添加instruction到observation (VLN需要文本指令)
    obs['instruction'] = r2r_ep.instruction

    # 4. 评测循环
    for step in range(max_steps):
        # 4.1 通过policy获取动作
        action = self.policy.act(obs)

        # 4.2 通过env执行动作
        obs, reward, done, info = self.env.step(action)

        # 4.3 重新添加instruction到observation
        obs['instruction'] = r2r_ep.instruction

        # 4.4 检查是否完成
        if done:
            success = (info['distance_to_goal'] < success_threshold)
            break

    # 5. 返回结果
    return EpisodeResult(...)
```

#### 2.4.4 VLNEvaluator初始化

```python
class VLNEvaluator:
    def __init__(self,
                 env: Env,
                 policy: Policy,
                 default_max_steps: int = 50,
                 default_success_threshold: float = 0.2):
        self.env = env
        self.policy = policy
        self.default_max_steps = default_max_steps
        self.default_success_threshold = default_success_threshold
```

#### 2.4.5 使用示例

```python
# 使用Mock组件进行测试
from vln_evaluator import VLNEvaluator, MockEnv
from vln_evaluator.policy import MockPolicy

env = MockEnv()
policy = MockPolicy()
evaluator = VLNEvaluator(env, policy)
results = evaluator.evaluate_dataset('datasets/sample_r2r.json')

# 使用O3DE + WebSocket VLM
from vln_evaluator.env import O3DEEnv
from vln_evaluator.policy import WebSocketPolicy

env = O3DEEnv(simulator_config={'mode': 'socket'})
policy = WebSocketPolicy('localhost', '8080')
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    evaluator.save_results(results, 'results/eval.json')
```

---

## 三、数据流图

### 3.1 整体数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                     VLN Evaluation Data Flow                     │
└─────────────────────────────────────────────────────────────────┘

数据集文件 (R2R JSON)
      │
      ▼
┌──────────────┐
│ Dataset      │ 加载episodes
│ Loader       │─────────────┐
└──────────────┘             │
                             ▼
                    ┌──────────────┐
                    │   Episode    │ episode_id, instruction,
                    │   Data       │ start/goal position
                    └──────────────┘
                             │
                             ▼
        ┌────────────────────────────────────────┐
        │         VLN Evaluator                 │
        │  ┌─────────────┐     ┌───────────────┐  │
        │  │    Env      │     │    Policy     │  │
        │  │ (Interface) │     │  (Interface)  │  │
        │  └─────────────┘     └───────────────┘  │
        │         │                     │         │
        │         ▼                     ▼         │
        │    ┌──────────────────────────────┐    │
        │    │    Evaluation Loop           │    │
        │    │  ┌────────────────────────┐  │    │
        │    │  │ obs = env.reset(ep)    │  │    │
        │    │  │ obs['instruction'] =   │  │    │
        │    │  │       ep.instruction    │  │    │
        │    │  └────────────────────────┘  │    │
        │    │           │                  │    │
        │    │           ▼                  │    │
        │    │  ┌────────────────────────┐  │    │
        │    │  │ action =               │  │    │
        │    │  │   policy.act(obs)      │  │    │
        │    │  └────────────────────────┘  │    │
        │    │           │                  │    │
        │    │           ▼                  │    │
        │    │  ┌────────────────────────┐  │    │
        │    │  │ obs, reward, done,     │  │    │
        │    │  │   info =               │  │    │
        │    │  │   env.step(action)     │  │    │
        │    │  └────────────────────────┘  │    │
        │    │           │                  │    │
        │    │           ▼                  │    │
        │    │  ┌────────────────────────┐  │    │
        │    │  │ obs['instruction'] =   │  │    │
        │    │  │       ep.instruction    │  │    │
        │    │  └────────────────────────┘  │    │
        │    └──────────────────────────────┘    │
        │         │                  │           │
        └─────────┼──────────────────┼───────────┘
                  │                  │
                  ▼                  ▼
        ┌─────────────────┐  ┌──────────────┐
        │  O3DE Simulator │  │  Remote VLM  │
        │  (内部实现)      │  │  Server      │
        └─────────────────┘  └──────────────┘
                             │
                             ▼
                    ┌──────────────┐
                    │ Evaluation   │ summary + episodes
                    │   Result     │─────────────┐
                    └──────────────┘             │
                                                 ▼
                                          ┌──────────────┐
                                          │ JSON Output  │
                                          └──────────────┘
```

### 3.2 单步数据流

```
┌────────────────────────────────────────────────────────────────┐
│                    Single Step Data Flow                        │
└────────────────────────────────────────────────────────────────┘

时间 t
  │
  ▼
┌─────────────────┐     ┌─────────────────┐
│  O3DE Simulator │     │  ROS2 Topics    │
│                 │     │                 │
│  /rgb ──────────┼────►│  Image msg      │
│  /depth ────────┼────►│  Image msg      │
│  /scan ─────────┼────►│  LaserScan msg  │
└─────────────────┘     └─────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────────────────────────────┐
│      SensorSubscriber                   │
│  ┌──────────────┐  ┌─────────────────┐ │
│  │ latest_rgb   │  │ latest_depth    │ │
│  │ (ndarray)    │  │ (ndarray)       │ │
│  └──────────────┘  └─────────────────┘ │
└─────────────────────────────────────────┘
         │
         ▼ obs = {'rgb': ..., 'depth': ...}
┌─────────────────────────────────────────┐
│      VLM Client                         │
│  ┌─────────────────────────────────┐   │
│  │ act(rgb, depth, instruction)   │   │
│  └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
         │
         ▼ action (0/1/2/3)
┌─────────────────────────────────────────┐
│      Action Executor                     │
│  ┌─────────────────────────────────┐   │
│  │ execute_action(action)          │   │
│  │   │                             │   │
│  │   ├── forward_25cm()            │   │
│  │   ├── rotate_15deg('left')      │   │
│  │   ├── rotate_15deg('right')     │   │
│  │   └── stop()                    │   │
│  └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
         │
         ▼ /cmd_vel (Twist msg)
┌─────────────────────────────────────────┐
│      O3DE Physics Engine                 │
│  (Robot moves)                           │
└─────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────┐
│      O3DE API Query                     │
│  ┌─────────────────────────────────┐   │
│  │ tc.get_world_position(robot_id) │   │
│  └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
         │
         ▼ position = {'x': ..., 'y': ..., 'z': ...}
┌─────────────────────────────────────────┐
│      Success Judge                       │
│  ┌─────────────────────────────────┐   │
│  │ distance = euclidean(pos, goal) │   │
│  │ success = (action==0 and dist<  │   │
│  │            threshold)            │   │
│  └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
         │
         ▼ info = {position, distance_to_goal, collision, step}
┌─────────────────────────────────────────┐
│      Return to Evaluator                 │
│  obs, reward, done, info                 │
└─────────────────────────────────────────┘

时间 t+1 (循环继续)
```

---

## 四、关键代码片段

### 4.1 Episode定义

```python
# o3de_simulator.py
class Episode:
    episode_id: str
    scene_id: str
    start_position: Dict[str, float]
    start_rotation: Dict[str, float]  # 欧拉角(度)
    goal_position: Dict[str, float]
    instruction: str
    scene_config_path: Optional[str] = None
    max_steps: Optional[int] = None
```

### 4.2 Simulator Reset流程

```python
def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
    # 1. 保存episode信息
    self.current_episode = episode
    self.current_step = 0
    self.goal_position = episode.goal_position
    self.trajectory = []

    # 2. 重置碰撞状态
    self.sensor_sub.reset_collision_state()

    # 3. 加载场景
    if episode.scene_config_path:
        self.o3de_manager.create_scene(episode.scene_config_path)

    # 4. 创建/设置机器人
    if self.o3de_manager.robot_entity_id is None:
        self.o3de_manager.create_robot(
            name='robot',
            position=episode.start_position,
            rotation=episode.start_rotation
        )
    else:
        self.o3de_manager.set_robot_position(
            episode.start_position,
            episode.start_rotation
        )

    # 5. 等待传感器就绪
    time.sleep(0.5)

    # 6. 获取初始观察
    obs = self.sensor_sub.get_observation()

    # 7. 记录初始位置
    pos = self.o3de_manager.get_robot_position()
    if pos:
        self.trajectory.append(pos)

    return obs
```

### 4.3 Simulator Step流程

```python
def step(self, action: int) -> Tuple[obs, reward, done, info]:
    # 1. 执行动作
    self.action_executor.execute_action(action)

    # 2. 等待动作完成和传感器更新
    time.sleep(0.1)
    for _ in range(5):
        self.sensor_sub.spin_once()

    # 3. 获取观察
    obs = self.sensor_sub.get_observation()

    # 4. 通过O3DE API查询位姿
    position = self.o3de_manager.get_robot_position()

    # 5. 计算距离
    distance_to_goal = euclidean_distance(position, self.goal_position)

    # 6. 检查碰撞
    collision = self.sensor_sub.collision_detected

    # 7. 更新步数
    self.current_step += 1

    # 8. 记录轨迹
    if position:
        self.trajectory.append(position)

    # 9. 判断done
    done = False
    if action == 0 and distance_to_goal < self.success_threshold:
        done = True

    if self.current_episode.max_steps:
        if self.current_step >= self.current_episode.max_steps:
            done = True

    # 10. 构建info
    info = {
        'position': position,
        'distance_to_goal': distance_to_goal,
        'collision': collision,
        'collision_count': self.sensor_sub.get_collision_count(),
        'step': self.current_step
    }

    return obs, 0.0, done, info
```

### 4.4 评测循环

```python
def evaluate_episode(self, r2r_ep: R2REpisode) -> EpisodeResult:
    # 转换episode
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

    # Reset
    obs = self.simulator.reset(episode)

    # Loop
    for step in range(max_steps):
        # VLM inference
        action = self.vlm_client.act(obs['rgb'], obs['depth'], r2r_ep.instruction)

        # Execute
        obs, reward, done, info = self.simulator.step(action)

        # Check done
        if done:
            success = (info['distance_to_goal'] < success_threshold)
            break

    # Return result
    return EpisodeResult(
        episode_id=r2r_ep.episode_id,
        success=success,
        final_distance_to_goal=info['distance_to_goal'],
        steps=step + 1,
        collision_count=info['collision_count'],
        trajectory=self.simulator.get_trajectory()
    )
```

---

## 五、配置说明

### 5.1 配置文件结构 (vln_eval.yaml)

```yaml
# O3DE Simulator配置
simulator:
  mode: socket                      # 'peb' 或 'socket'
  success_threshold: 0.2            # 成功距离阈值(米)
  collision_threshold: 0.3          # 碰撞检测阈值(米)
  linear_speed: 0.1                 # 线速度(m/s)
  angular_speed: 0.3                # 角速度(rad/s)

  # Socket模式配置
  socket_host: 127.0.0.1
  socket_port: 8080

# VLM配置
vlm:
  url: ws://localhost:8080          # WebSocket URL
  use_pickle: true                  # 序列化方式

# 评测配置
evaluation:
  default_max_steps: 50
  default_success_threshold: 0.2

  output_dir: results
  save_trajectories: true

# 日志配置
logging:
  level: INFO
  log_file: logs/vln_eval.log
```

### 5.2 配置优先级

```
命令行参数 > YAML配置文件 > 代码默认值
```

示例：
```bash
# 命令行参数覆盖
python run_vln_eval.py \
    --vlm-url ws://192.168.1.100:9999 \  # 覆盖YAML中的url
    --sim-mode socket                      # 覆盖YAML中的mode
```

---

## 六、运行流程

### 6.1 完整执行流程

```
1. 启动O3DE仿真器
   ├─ 加载场景
   ├─ 创建机器人entity
   └─ 启动ROS2传感器发布

2. 启动VLM服务器
   ├─ 监听WebSocket端口
   └─ 等待评测请求

3. 运行评测脚本
   ├─ 加载配置文件
   ├─ 初始化Simulator
   ├─ 连接VLM服务器
   ├─ 加载数据集
   └─ 执行评测循环

4. 评测循环 (每个episode)
   ├─ reset(episode)
   │   ├─ 加载场景
   │   ├─ 设置机器人位置
   │   └─ 等待传感器就绪
   ├─ for step in range(max_steps):
   │   ├─ 获取RGB/Depth
   │   ├─ 调用VLM获取动作
   │   ├─ 执行动作
   │   ├─ 查询位姿
   │   ├─ 判断成功
   │   └─ 记录轨迹
   └─ 保存episode结果

5. 生成汇总报告
   ├─ 计算统计指标
   ├─ 保存JSON文件
   └─ 打印摘要

6. 清理资源
   ├─ 关闭simulator
   ├─ 断开VLM连接
   └─ 关闭ROS2节点
```

### 6.2 时间线

```
t=0:     初始化系统
t=1s:    连接VLM服务器
t=2s:    加载数据集
t=3s:    开始episode 1
t=3.5s:  reset simulator
t=4s:    开始step loop
t=4.5s:  step 1 (get obs + vlm inference + execute)
t=5s:    step 2
...
t=30s:   episode 1 完成
t=30.5s: 开始episode 2
...
t=180s:  所有episode完成
t=181s:  保存结果
t=182s:  清理资源
```

---

## 七、依赖关系

### 7.1 模块依赖图

```
┌─────────────────────────────────────────────────────────────┐
│                    run_vln_eval.py                         │
│                    (entry point)                           │
└────────────────────────┬────────────────────────────────────┘
                         │
         ┌───────────────┼───────────────┐
         ▼               ▼               ▼
┌────────────────┐ ┌──────────────┐ ┌──────────────┐
│ vln_evaluator/ │ │ configs/      │ │ datasets/    │
└───────┬────────┘ └──────────────┘ └──────────────┘
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│                  vln_evaluator/                           │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────────┐  │
│  │dataset_     │ │ws_policy_    │ │vln_evaluator.py  │  │
│  │loader.py    │ │client.py     │ └────────┬─────────┘  │
│  └─────────────┘ └──────────────┘          │            │
└───────────────────────────────────────────┼──────────────┘
                                               │
                   ┌───────────────────────────┼──────────────┐
                   ▼                           ▼              ▼
          ┌──────────────────┐    ┌──────────────────┐ ┌────────────┐
          │o3de_simulator.py │    │action_executor.py│ │ policy.py  │
          └────────┬─────────┘    └──────────────────┘ └────────────┘
                   │
         ┌─────────┼─────────┐
         ▼         ▼         ▼
    ┌────────┐ ┌──────┐ ┌──────────┐
    │ o3de_  │ │ rclpy│ │cv_bridge │
    │ sim/   │ │      │ │          │
    └────────┘ └──────┘ └──────────┘
```

### 7.2 外部依赖

| 依赖包 | 版本要求 | 用途 |
|--------|---------|------|
| o3de_sim | - | O3DE Python API |
| rclpy | - | ROS2 Python客户端 |
| sensor_msgs | - | ROS2消息类型 |
| cv_bridge | - | ROS2-OpenCV桥接 |
| websocket-client | - | WebSocket通信 |
| numpy | - | 数组处理 |
| pyyaml | - | YAML配置解析 |
| dataclasses | Python 3.7+ | 数据类 |

### 7.3 安装依赖

```bash
# ROS2 (已安装O3DE的情况下通常已包含)
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-sensor-msgs

# Python包
pip install websocket-client pyyaml numpy
```

---

## 八、扩展开发指南

### 8.1 添加新的动作类型

1. 在 `o3de_simulator.py` 中扩展 `ActionType`:
```python
class ActionType(Enum):
    STOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3
    BACKWARD = 4  # 新增
```

2. 在 `ActionExecutorWrapper` 中实现新动作:
```python
def execute_backward_25cm(self):
    self.node.get_logger().info('Executing: BACKWARD 25cm')
    self._move_timed(linear_x=-self.linear_speed,
                    duration_sec=0.25/self.linear_speed)
```

3. 更新 `execute_action` 方法:
```python
def execute_action(self, action: int):
    if action == ActionType.BACKWARD.value:
        self.execute_backward_25cm()
    # ...
```

### 8.2 添加新的评测指标

1. 扩展 `EvaluationResult`:
```python
@dataclass
class EvaluationResult:
    # 现有字段...
    spl: float = 0.0  # 新增SPL指标
    oracle_success: int = 0  # 新增oracle成功数
```

2. 在 `VLNEvaluator.evaluate_dataset` 中计算:
```python
# 计算SPL
for ep in results.episodes:
    shortest_path = calculate_shortest_path(ep.start_pos, ep.goal_pos)
    actual_path = len(ep.trajectory)
    ep.spl = (ep.success * shortest_path) / max(actual_path, shortest_path)

results.spl = np.mean([ep.spl for ep in results.episodes])
```

### 8.3 添加轨迹可视化

创建新文件 `vln_evaluator/visualizer.py`:

```python
import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(episode_result: EpisodeResult, save_path: str):
    """绘制轨迹图"""
    trajectory = episode_result.trajectory
    x = [p['x'] for p in trajectory]
    z = [p['z'] for p in trajectory]

    plt.figure(figsize=(10, 8))
    plt.plot(x, z, 'b-', label='Trajectory', linewidth=2)
    plt.plot(x[0], z[0], 'go', label='Start', markersize=10)
    plt.plot(episode_result.goal_position['x'],
             episode_result.goal_position['z'],
             'r*', label='Goal', markersize=15)

    plt.xlabel('X (m)')
    plt.ylabel('Z (m)')
    plt.title(f'Trajectory: {episode_result.episode_id}')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
```

### 8.4 支持多场景并行评测

```python
from multiprocessing import Process, Queue

def evaluate_single_process(dataset_path, config, result_queue, process_id):
    """单个评测进程"""
    evaluator = VLNEvaluator(**config)
    # ... 评测逻辑
    result_queue.put(results)

def parallel_evaluate(dataset_path, config, num_processes=4):
    """并行评测"""
    # 分割数据集
    episodes = load_dataset(dataset_path)
    chunks = split_chunks(episodes, num_processes)

    # 启动进程
    processes = []
    result_queue = Queue()
    for i, chunk in enumerate(chunks):
        p = Process(target=evaluate_single_process,
                   args=(chunk, config, result_queue, i))
        p.start()
        processes.append(p)

    # 等待完成
    for p in processes:
        p.join()

    # 合并结果
    results = []
    while not result_queue.empty():
        results.append(result_queue.get())

    return merge_results(results)
```

### 8.5 调试技巧

**启用调试日志**:
```python
import logging
logging.basicConfig(level=logging.DEBUG)

# 在代码中添加日志
self.node.get_logger().info('Message')
self.node.get_logger().debug(f'Debug info: {var}')
```

**可视化中间结果**:
```python
# 保存每步的RGB图像
from PIL import Image
for step, obs in enumerate(observations):
    rgb = Image.fromarray(obs['rgb'])
    rgb.save(f'debug_rgb_step_{step}.png')
```

**检查ROS2话题**:
```bash
# 查看所有话题
ros2 topic list

# 查看话题内容
ros2 topic echo /rgb
ros2 topic hz /rgb

# 检查QoS
ros2 topic info /rgb -v
```

---

## 附录

### A. 常见问题

**Q1: 传感器数据超时**
```
TimeoutError: Timeout waiting for sensor data
```
**A**: 检查O3DE是否正确发布ROS2话题：
```bash
ros2 topic list  # 应该包含/rgb, /depth, /scan
ros2 topic hz /rgb  # 检查发布频率
```

**Q2: VLM连接失败**
```
ConnectionError: Failed to connect to ws://localhost:8080
```
**A**: 确认VLM服务器是否运行：
```bash
# 测试WebSocket连接
python -c "import websocket; ws=websocket.create_connection('ws://localhost:8080')"
```

**Q3: 机器人不动**
```
Robot doesn't move when action is executed
```
**A**: 检查/cmd_vel话题：
```bash
ros2 topic echo /cmd_vel  # 应该能看到消息
# 检查O3DE中是否正确订阅/cmd_vel
```

### B. 性能优化建议

1. **使用Pickle序列化**: 比JSON快约3-5倍
2. **减少等待时间**: 调整`step()`中的等待次数
3. **批量评测**: 一次性加载所有场景，减少reset开销
4. **异步通信**: 使用异步WebSocket客户端

### C. 参考资料

- [O3DE Documentation](https://o3de.org/docs/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [R2R Dataset Paper](https://arxiv.org/abs/1812.07173)
- [VLN Challenge](https://eval.ai/web/challenges/challenge-page/327/overview)
