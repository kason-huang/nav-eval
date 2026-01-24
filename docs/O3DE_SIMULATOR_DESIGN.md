# O3DE Simulator 设计文档

## 一、概述

本文档描述 `o3de_simulator.py` 的设计，该模块将O3DE仿真环境封装成一个标准的仿真器接口，供VLN评测系统使用。

## 二、设计目标

1. **统一接口**：提供Habitat风格的Env接口（reset/step/close）
2. **O3DE封装**：封装o3de_sim API，隐藏底层实现细节
3. **ROS2集成**：集成ROS2传感器订阅和动作执行
4. **易于使用**：简化VLN评测流程，使用者无需关心O3DE细节

## 三、接口定义

### 3.1 抽象接口（env.py）

```python
import abc

class Env(abc.ABC):
    """环境抽象基类，遵循Habitat-lab约定"""

    @abc.abstractmethod
    def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
        """重置环境

        Returns:
            obs: {'rgb': ndarray, 'depth': ndarray}
        """
        pass

    @abc.abstractmethod
    def step(self, action: int) -> Tuple[obs, reward, done, info]:
        """执行一步

        Returns:
            obs: {'rgb': ndarray, 'depth': ndarray}
            reward: float (0.0 for VLN)
            done: bool
            info: dict with position, distance_to_goal, collision_count, trajectory
        """
        pass

    @abc.abstractmethod
    def close(self):
        """关闭环境"""
        pass
```

**Env继承层次结构**:
```
Env (abc.ABC)
  ├── MockEnv - 用于测试，无O3DE/ROS2依赖
  └── O3DEEnv - 生产环境，封装O3DESimulator
       └── (内部) O3DESimulator - 实际的O3DE仿真器实现
```

**注意**: O3DESimulator是内部实现，不直接暴露给VLNEvaluator。VLNEvaluator通过Env接口访问环境。

### 3.2 O3DE Simulator接口

```python
class O3DESimulator(Env):
    """O3DE仿真器实现"""

    def __init__(self, mode=constants.O3DE_AGENT_SOCKET_MODE,
                 sensor_qos=None):
        """
        初始化O3DE仿真器

        Args:
            mode: O3DE通信模式
                - constants.O3DE_AGENT_PEB_MODE: 进程内通信
                - constants.O3DE_AGENT_SOCKET_MODE: Socket通信
            sensor_qos: ROS2传感器QoS配置（可选）
        """
        pass

    def reset(self, episode: Episode) -> dict:
        """
        重置仿真环境

        Args:
            episode: Episode对象，包含:
                - episode_id: str
                - scene_id: str (用于查找场景配置)
                - start_position: dict or Vector3
                - start_rotation: dict or Euler (度)
                - goal_position: dict or Vector3
                - instruction: str (导航文本指令)

        Returns:
            obs: 初始观察
                {
                    'rgb': ndarray(H, W, 3),    # RGB图像
                    'depth': ndarray(H, W)       # 深度图像
                }
        """
        pass

    def step(self, action: int) -> tuple:
        """
        执行动作

        Args:
            action: 动作ID
                - 0: STOP (停止)
                - 1: FORWARD (前进25cm)
                - 2: LEFT (左转15°)
                - 3: RIGHT (右转15°)

        Returns:
            obs: {'rgb': ndarray, 'depth': ndarray}
            reward: float (VLN不使用，固定返回0.0)
            done: bool (episode是否结束)
            info: dict {
                'position': Vector3,        # 当前位置
                'distance_to_goal': float,  # 到目标的距离
                'collision': bool,          # 是否发生碰撞
                'step': int                 # 当前步数
            }
        """
        pass

    def close(self):
        """关闭仿真器，清理资源"""
        pass

    # ===== 辅助方法 =====
    def get_current_position(self) -> 'Vector3':
        """获取机器人当前位置（通过O3DE API）"""
        pass

    def get_distance_to_goal(self) -> float:
        """计算到目标的欧氏距离"""
        pass

    def get_robot_entity_id(self) -> 'EntityId':
        """获取机器人entity ID"""
        pass
```

## 四、Episode数据结构

```python
from dataclasses import dataclass
from typing import Optional
import numpy as np

@dataclass
class Episode:
    """Episode数据结构"""
    episode_id: str
    scene_id: str
    start_position: dict  # {'x': 0.0, 'y': 0.0, 'z': 0.0}
    start_rotation: dict  # {'x': 0, 'y': 0, 'z': 0} 欧拉角(度)
    goal_position: dict   # {'x': 5.0, 'y': 0.0, 'z': 3.0}
    instruction: str

    # 可选字段
    scene_config_path: Optional[str] = None
    max_steps: Optional[int] = None
```

## 五、架构设计

### 5.1 整体架构

```
┌─────────────────────────────────────────────────────────────┐
│                     O3DESimulator                           │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  O3DE Agent (o3de_sim)                               │  │
│  │  - entity_tool, tc, pc, render_tool, etc.            │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Action Executor (集成)                              │  │
│  │  - ROS2 /cmd_vel 发布                                │  │
│  │  - execute_forward_25cm()                            │  │
│  │  - execute_rotate_15deg()                            │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  Sensor Subscriber                                   │  │
│  │  - ROS2 /rgb, /depth 订阅                            │  │
│  │  - ROS2 /scan 订阅 (碰撞检测)                        │  │
│  │  - 缓存最新数据                                       │  │
│  └──────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  State Manager                                       │  │
│  │  - 当前步数                                          │  │
│  │  - 碰撞状态                                          │  │
│  │  - 目标位置                                          │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
    ┌─────────┐        ┌─────────┐         ┌─────────┐
    │  O3DE   │        │  ROS2   │         │External │
    │Engine   │        │ Topics  │         │Evaluator│
    └─────────┘        └─────────┘         └─────────┘
```

### 5.2 模块分解

#### 5.2.1 O3DEAgent管理

```python
class O3DEAgentManager:
    """管理O3DE Agent连接"""

    def __init__(self, mode):
        from o3de_sim.o3de_api import o3de_agent
        self.agent = o3de_agent.O3DEAgent(mode=mode)
        self.robot_entity_id = None

    def create_scene(self, scene_config_path: str):
        """加载场景配置"""
        # 使用BasicSceneBuilder或直接API
        pass

    def create_robot(self, position, rotation):
        """创建机器人entity"""
        pass

    def set_robot_position(self, position, rotation):
        """设置机器人位姿"""
        pass

    def get_robot_position(self):
        """获取机器人位置"""
        return self.agent.tc.get_position(self.robot_entity_id)
```

#### 5.2.2 动作执行器

```python
class ActionExecutorWrapper:
    """封装action_executor"""

    def __init__(self, robot_entity_id, o3de_agent):
        # 集成action_executor的逻辑
        # 但通过o3de_sim API而非直接ROS2
        pass

    def execute_action(self, action: int):
        """执行动作"""
        if action == 0:
            self.stop()
        elif action == 1:
            self.forward_25cm()
        elif action == 2:
            self.rotate_15deg('left')
        elif action == 3:
            self.rotate_15deg('right')

    def forward_25cm(self):
        """前进25cm - 通过o3de_sim API"""
        # 方案1: 直接设置transform（精确但跳过物理）
        # 方案2: 通过ROS2 /cmd_vel控制（有物理效果）
        pass

    def rotate_15deg(self, direction):
        """旋转15度"""
        pass
```

#### 5.2.3 传感器订阅器

```python
class SensorSubscriber:
    """ROS2传感器订阅器"""

    def __init__(self, qos_profile):
        import rclpy
        from sensor_msgs.msg import Image, LaserScan

        rclpy.init()
        self.node = rclpy.create_node('sensor_subscriber')

        # RGB订阅
        self.rgb_sub = self.node.create_subscription(
            Image, '/rgb', self.rgb_callback, qos_profile
        )
        self.latest_rgb = None

        # Depth订阅
        self.depth_sub = self.node.create_subscription(
            Image, '/depth', self.depth_callback, qos_profile
        )
        self.latest_depth = None

        # Scan订阅 (碰撞检测)
        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile
        )
        self.collision_detected = False
        self.collision_count = 0

    def rgb_callback(self, msg):
        # 转换ROS2 Image到numpy array
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def scan_callback(self, msg):
        # 碰撞检测逻辑 (复用action_executor)
        ranges = msg.ranges
        mid = len(ranges) // 2
        front_view = ranges[mid-30:mid+30]
        valid_ranges = [r for r in front_view
                       if msg.range_min < r < msg.range_max]
        if valid_ranges and min(valid_ranges) < 0.3:
            self.collision_detected = True
            self.collision_count += 1

    def get_observation(self):
        """返回观察"""
        # 等待数据就绪
        while self.latest_rgb is None or self.latest_depth is None:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        return {
            'rgb': self.latest_rgb,
            'depth': self.latest_depth
        }
```

## 六、核心流程

### 6.1 初始化流程

```
__init__()
    │
    ├─► 初始化O3DE Agent (PEB/Socket)
    │
    ├─► 初始化ROS2节点
    │
    ├─► 创建传感器订阅器
    │   ├─► /rgb
    │   ├─► /depth
    │   └─► /scan
    │
    └─► 初始化状态变量
        ├─► current_step = 0
        ├─► goal_position = None
        └─► robot_entity_id = None
```

### 6.2 Reset流程

```
reset(episode)
    │
    ├─► 1. 加载场景
    │   └─► BasicSceneBuilder.build_scene(scene_config)
    │
    ├─► 2. 创建机器人entity
    │   ├─► editor_tool.create_new_entity_at_position()
    │   ├─► 添加Mesh组件
    │   └─► 保存robot_entity_id
    │
    ├─► 3. 设置机器人初始位姿
    │   ├─► tc.set_position(start_position)
    │   └─► tc.set_rotation(euler_to_quaternion(start_rotation))
    │
    ├─► 4. 设置目标位置
    │   └─► goal_position = episode.goal_position
    │
    ├─► 5. 重置状态
    │   ├─► current_step = 0
    │   ├─► collision_count = 0
    │   └─► collision_detected = False
    │
    ├─► 6. 等待传感器数据就绪
    │   └─► spin_until_data_ready()
    │
    └─► 7. 返回初始observation
        └─► {'rgb': ..., 'depth': ...}
```

### 6.3 Step流程

```
step(action)
    │
    ├─► 1. 执行动作
    │   └─► action_executor.execute_action(action)
    │       ├─► 发布 /cmd_vel
    │       ├─► 等待动作完成 (基于/odom或时间)
    │       └─► 停止机器人
    │
    ├─► 2. 等待传感器更新
    │   └─► rclpy.spin_once()
    │
    ├─► 3. 获取observation
    │   └─► {'rgb': latest_rgb, 'depth': latest_depth}
    │
    ├─► 4. 通过O3DE API查询位姿
    │   └─► current_pos = tc.get_position(robot_entity_id)
    │
    ├─► 5. 计算距离
    │   └─► distance = euclidean(current_pos, goal_position)
    │
    ├─► 6. 判断done
    │   ├─► done = (action == 0 and distance < threshold)
    │   └─► 或超时/碰撞等条件
    │
    ├─► 7. 更新步数
    │   └─► current_step += 1
    │
    └─► 8. 返回 (obs, reward, done, info)
        └─► info = {
                'position': current_pos,
                'distance_to_goal': distance,
                'collision': collision_detected,
                'step': current_step
            }
```

### 6.4 Close流程

```
close()
    │
    ├─► 停止机器人
    │   └─► cmd_pub.publish(Twist())
    │
    ├─► 销毁ROS2节点
    │   └─► node.destroy_node()
    │
    └─► 关闭rclpy
        └─► rclpy.shutdown()
```

## 七、数据结构

### 7.1 Observation

```python
observation = {
    'rgb': np.ndarray,    # Shape: (H, W, 3), dtype: uint8
    'depth': np.ndarray   # Shape: (H, W), dtype: uint16
}
```

### 7.2 Info

```python
info = {
    'position': Vector3,       # 当前位置
    'distance_to_goal': float, # 到目标的欧氏距离
    'collision': bool,         # 本步是否发生碰撞
    'step': int                # 当前步数（从0开始）
}
```

### 7.3 Episode配置示例

```json
{
  "episode_id": "ep_001",
  "scene_id": "scene_001",
  "scene_config_path": "configs/scene_001.json",
  "start_position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "start_rotation": {"x": 0, "y": 0, "z": 0},
  "goal_position": {"x": 5.0, "y": 0.0, "z": 3.0},
  "instruction": "Walk straight and turn left...",
  "max_steps": 50
}
```

## 八、配置参数

```python
# O3DE Simulator配置
O3DE_SIMULATOR_CONFIG = {
    # O3DE连接模式
    'mode': 'socket',  # 'peb' or 'socket'

    # Socket配置（mode='socket'时）
    'socket_host': '127.0.0.1',
    'socket_port': 8080,

    # 传感器QoS
    'sensor_qos': {
        'reliability': 'BEST_EFFORT',
        'durability': 'VOLATILE',
        'depth': 10
    },

    # 成功判定阈值
    'success_threshold': 0.2,  # 米

    # 碰撞检测阈值
    'collision_threshold': 0.3,  # 米

    # 动作执行参数
    'linear_speed': 0.1,       # m/s
    'angular_speed': 0.3,      # rad/s

    # 机器人配置
    'robot_asset_path': 'Assets/Models/robot/robot.gltf',
    'robot_name': 'robot',
}
```

## 九、使用示例

### 9.1 通过Env接口使用（推荐）

**使用O3DEEnv进行评测**:
```python
from vln_evaluator.env import O3DEEnv
from vln_evaluator.policy import MockPolicy
from vln_evaluator import VLNEvaluator

# 创建环境（封装O3DESimulator）
env = O3DEEnv(simulator_config={'mode': 'socket'})

# 创建策略
policy = MockPolicy()

# 创建评测器
evaluator = VLNEvaluator(env=env, policy=policy)

# 评测数据集
results = evaluator.evaluate_dataset('datasets/sample_r2r.json')

# 保存结果
evaluator.save_results(results, 'results/eval.json')

# 清理
evaluator.close()
```

**使用上下文管理器**:
```python
from vln_evaluator.env import O3DEEnv
from vln_evaluator.policy import WebSocketPolicy
from vln_evaluator import VLNEvaluator

env = O3DEEnv(simulator_config={
    'mode': 'socket',
    'success_threshold': 0.2
})
policy = WebSocketPolicy('localhost', '8080')

with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    evaluator.save_results(results, 'results/eval.json')
```

### 9.2 直接使用O3DESimulator（高级用法）

**注意**: O3DESimulator是内部实现，通常通过O3DEEnv接口使用。直接使用适用于需要访问底层功能的场景。

```python
from o3de_simulator import O3DESimulator
from o3de_sim import constants

# 创建仿真器
sim = O3DESimulator(mode=constants.O3DE_AGENT_SOCKET_MODE)

# 创建episode
episode = sim.Episode(
    episode_id="ep_001",
    scene_id="scene_001",
    start_position={'x': 0, 'y': 0, 'z': 0},
    start_rotation={'x': 0, 'y': 0, 'z': 0},
    goal_position={'x': 5, 'y': 0, 'z': 3},
    instruction="Walk forward 5 meters"
)

# 重置环境
obs = sim.reset(episode)

# 评测循环
for step in range(50):
    # ... 获取model action ...
    action = model.act(obs['rgb'], obs['depth'], episode.instruction)

    # 执行动作
    obs, reward, done, info = sim.step(action)

    if done:
        print(f"Success! Distance: {info['distance_to_goal']}")
        break

# 关闭仿真器
sim.close()
```

### 9.3 使用MockEnv进行测试

```python
from vln_evaluator.env import MockEnv
from vln_evaluator.policy import MockPolicy
from vln_evaluator import VLNEvaluator

# 创建mock环境（无需O3DE/ROS2）
env = MockEnv()
policy = MockPolicy()

# 创建评测器
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    print(f"Success rate: {results.success_rate:.2%}")
```

## 十、实现注意事项

### 10.1 ROS2图像转换

使用cv_bridge或手动转换ROS2 Image消息到numpy array：

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def rgb_callback(self, msg: Image):
    self.latest_rgb = bridge.imgmsg_to_cv2(msg, 'bgr8')

def depth_callback(self, msg: Image):
    self.latest_depth = bridge.imgmsg_to_cv2(msg, '16UC1')
```

### 10.2 欧拉角与四元数转换

```python
import math
from o3de_sim.o3de_api.interface import math as o3de_math

def euler_to_quaternion(euler_deg):
    """欧拉角(度)转四元数"""
    x, y, z = [math.radians(d) for d in [euler_deg.x, euler_deg.y, euler_deg.z]]

    # ZYX顺序
    cy = math.cos(z * 0.5)
    sy = math.sin(z * 0.5)
    cp = math.cos(y * 0.5)
    sp = math.sin(y * 0.5)
    cr = math.cos(x * 0.5)
    sr = math.sin(x * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return o3de_math.Quaternion(qw, qx, qy, qz)
```

### 10.3 同步O3DE更新

在执行动作后需要等待O3DE处理：

```python
def step(self, action):
    # 执行动作
    self.action_executor.execute_action(action)

    # 等待O3DE更新
    self.o3de_agent.general.wait_for_propagation()

    # 等待ROS2传感器更新
    for _ in range(5):  # 等待几帧
        rclpy.spin_once(self.sensor_sub.node, timeout_sec=0.01)

    # 获取新状态
    ...
```

### 10.4 避免重复初始化ROS2

如果整个评测过程只创建一个simulator实例：

```python
def __init__(self):
    # 检查rclpy是否已初始化
    try:
        rclpy.get_context().ok()
    except:
        rclpy.init()
```

## 十一、测试计划

### 11.1 单元测试

```python
# test_o3de_simulator.py

def test_simulator_init():
    """测试初始化"""
    sim = O3DESimulator()
    assert sim.robot_entity_id is None
    assert sim.current_step == 0

def test_reset():
    """测试reset"""
    sim = O3DESimulator()
    episode = create_test_episode()
    obs = sim.reset(episode)

    assert 'rgb' in obs
    assert 'depth' in obs
    assert obs['rgb'].shape == (H, W, 3)

def test_step():
    """测试step"""
    sim = O3DESimulator()
    episode = create_test_episode()
    obs = sim.reset(episode)

    obs, reward, done, info = sim.step(action=1)

    assert reward == 0.0
    assert 'position' in info
    assert 'distance_to_goal' in info

def test_position_query():
    """测试位姿查询"""
    sim = O3DESimulator()
    episode = create_test_episode()
    sim.reset(episode)

    pos = sim.get_current_position()
    assert isinstance(pos, o3de_math.Vector3)
```

### 11.2 集成测试

```python
def test_full_episode():
    """完整episode测试"""
    sim = O3DESimulator()
    episode = create_simple_episode()

    obs = sim.reset(episode)

    for _ in range(10):
        obs, reward, done, info = sim.step(1)  # 前进
        if done:
            break

    assert info['step'] > 0
    sim.close()
```

## 十二、文件结构

```
nav-eval/
├── sim.py                    # (可选) 抽象接口/入口
├── o3de_simulator.py         # O3DESimulator主实现
├── action_executor.py        # 现有，将被集成
├── policy.py                 # 现有
├── env.py                    # 现有抽象基类
├── o3de_sim/                 # 现有
│   └── o3de_api/
└── docs/
    └── O3DE_SIMULATOR_DESIGN.md  # 本文档
```

## 十三、后续扩展

1. **多机器人支持**：支持同时创建多个机器人
2. **轨迹可视化**：集成matplotlib/plotly生成轨迹图
3. **性能优化**：批量处理、异步IO
4. **日志记录**：详细记录每步的传感器数据
5. **断点续评**：支持保存状态和恢复
