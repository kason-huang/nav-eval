# VLN Evaluation System - O3DE

基于O3DE仿真环境的视觉-语言导航（VLN）评测系统。

## 项目简介

本系统提供完整的VLN评测流程，采用**Env/Policy接口设计**，遵循Habitat-lab约定：

- **O3DE仿真环境**：基于Open 3D Engine的物理仿真
- **Env接口**：标准环境接口（reset/step/close），封装仿真器内部实现
- **Policy接口**：动作选择接口，支持本地和远程VLM
- **ROS2集成**：通过ROS2话题获取传感器数据和控制机器人
- **R2R数据集**：支持R2R格式的导航指令数据集

## 架构设计

系统采用**Env-Policy分离架构**：

```
┌─────────────────────────────────────────────────────────────────┐
│                        VLNEvaluator                             │
│  ┌──────────────────┐    ┌──────────────────┐                   │
│  │ Dataset Loader   │    │  Result Manager  │                   │
│  └──────────────────┘    └──────────────────┘                   │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Evaluation Loop                             │   │
│  │    obs = env.reset(episode)                             │   │
│  │    action = policy.act(obs)                              │   │
│  │    obs, reward, done, info = env.step(action)            │   │
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
```

**关键特性**：
- **环境抽象**：Env接口隐藏O3DE细节，可轻松替换仿真器
- **策略抽象**：Policy接口支持不同VLM实现
- **测试友好**：MockEnv/MockPolicy无需O3DE即可测试

## 系统要求

### 硬件要求
- CPU: 4核心以上
- 内存: 8GB以上
- GPU: 支持O3DE的显卡（可选）

### 软件要求
- **操作系统**: Ubuntu 20.04/22.04 或 Windows 10/11
- **O3DE**: Open 3D Engine（[下载地址](https://o3de.org/download/)）
- **ROS2**: Humble 或 Foxy
- **Python**: 3.8+

### ROS2安装（如果未安装）

```bash
# Ubuntu 22.04 + ROS2 Humble
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-rosdep
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs ros-humble-geometry-msgs
```

## 安装依赖

### 1. 克隆项目

```bash
cd /path/to/your/workspace
git clone <repository-url> nav-eval
cd nav-eval
```

### 2. 安装Python依赖

```bash
pip install websocket-client pyyaml numpy opencv-python
```

### 3. 配置环境变量

```bash
# 添加O3DE Python路径到PYTHONPATH
export PYTHONPATH=/path/to/o3de/python:$PYTHONPATH

# 添加项目路径
export PYTHONPATH=/path/to/nav-eval:$PYTHONPATH
```

建议将其添加到 `~/.bashrc`:

```bash
echo 'export PYTHONPATH=/path/to/o3de/python:/path/to/nav-eval:$PYTHONPATH' >> ~/.bashrc
source ~/.bashrc
```

## 项目结构

```
nav-eval/
├── docs/                           # 设计文档
│   ├── VLN_EVALUATION_DESIGN.md    # VLN评测系统设计
│   ├── O3DE_SIMULATOR_DESIGN.md    # O3DE Simulator设计
│   └── IMPLEMENTATION_DETAILS.md   # 实现细节
├── vln_evaluator/                  # 评测器模块包
│   ├── __init__.py                 # 模块导出
│   ├── env.py                      # Env接口（Env, MockEnv, O3DEEnv）
│   ├── policy.py                   # Policy接口（Policy, MockPolicy, WebSocketPolicy）
│   ├── dataset_loader.py           # R2R数据集加载器
│   └── vln_evaluator.py            # 主评测器
├── o3de_sim/                       # O3DE仿真接口库
│   └── o3de_api/
├── o3de_simulator.py               # O3DE仿真器（内部实现）
├── action_executor.py              # ROS2动作执行器（被O3DESimulator使用）
├── run_vln_eval.py                 # 评测运行脚本
├── configs/
│   └── vln_eval.yaml               # 配置文件
├── datasets/
│   └── sample_r2r.json            # 示例数据集
├── tests/                          # 测试套件
│   ├── test_evaluation.py          # 主测试文件
│   └── mock_vlm_server.py          # Mock VLM服务器
└── results/                        # 结果输出目录
```

## 快速开始

### 使用Mock组件测试（无需O3DE）

```python
from vln_evaluator import VLNEvaluator, MockEnv, MockPolicy

# 创建Mock组件
env = MockEnv()
policy = MockPolicy()

# 创建评测器
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    print(f"Success Rate: {results.success_rate:.2%}")
```

### 使用O3DE + WebSocket VLM

```python
from vln_evaluator import VLNEvaluator, O3DEEnv
from vln_evaluator.policy import WebSocketPolicy

# 创建环境（O3DE）
env = O3DEEnv(simulator_config={'mode': 'socket'})

# 创建策略（WebSocket VLM）
policy = WebSocketPolicy('localhost', '8080')

# 创建评测器
with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    evaluator.save_results(results, 'results/eval.json')
```

## 运行准备

### 步骤1: 准备O3DE场景

1. 启动O3DE Editor
2. 创建或加载场景
3. 添加机器人entity（名称为"robot"）
4. 添加ROS2传感器组件：
   - ROS2 Camera Sensor (RGB)
   - ROS2 Camera Sensor (Depth)
   - ROS2 LaserScan Sensor
5. 启用物理模拟
6. 进入Game Mode，验证ROS2话题发布

### 步骤2: 验证ROS2话题

```bash
# 检查话题列表
ros2 topic list

# 应该看到以下话题：
# /rgb, /depth, /scan, /cmd_vel, /odom

# 检查话题数据
ros2 topic hz /rgb
ros2 topic echo /rgb --once
```

### 步骤3: 启动VLM服务器

VLM服务器需要接收以下格式的观察并返回动作：

**输入格式**:
```python
{
    'rgb': ndarray(H, W, 3),      # RGB图像
    'depth': ndarray(H, W),       # 深度图像
    'instruction': str            # 导航指令
}
```

**输出格式**:
```python
0  # STOP
1  # FORWARD
2  # LEFT
3  # RIGHT
```

示例VLM服务器：
```python
import websocket
import pickle

def on_message(ws, message):
    obs = pickle.loads(message)
    # 调用VLM模型
    action = your_vlm_model(obs['rgb'], obs['depth'], obs['instruction'])
    ws.send(pickle.dumps(action))

server = websocket.WebSocketServer('0.0.0.0', 8080)
server.set_fn_message(on_message)
server.run_forever()
```

### 步骤4: 准备数据集

创建R2R格式的数据集JSON文件：

```json
{
  "episodes": [
    {
      "episode_id": "ep_001",
      "scene_id": "scene_001",
      "scene_config_path": "configs/scene_001.json",
      "instruction": "Walk straight forward for 3 meters",
      "start_position": {"x": 0.0, "y": 0.0, "z": 0.0},
      "start_rotation": {"x": 0, "y": 0, "z": 0},
      "goal_position": {"x": 3.0, "y": 0.0, "z": 0.0},
      "max_steps": 50
    }
  ]
}
```

## 运行评测

### 方式1: 使用命令行脚本

```bash
# 基本运行
python run_vln_eval.py \
    --dataset datasets/sample_r2r.json \
    --output results/eval_results.json

# 指定VLM服务器地址
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

### 方式2: 使用Python API（推荐）

```python
from vln_evaluator import VLNEvaluator, evaluate_vln
from vln_evaluator.env import O3DEEnv
from vln_evaluator.policy import WebSocketPolicy

# 使用便捷函数
env = O3DEEnv(simulator_config={'mode': 'socket'})
policy = WebSocketPolicy('localhost', '8080')

evaluate_vln(
    dataset_path='datasets/sample_r2r.json',
    output_path='results/eval_results.json',
    env=env,
    policy=policy
)
```

### 方式3: 完整控制

```python
from vln_evaluator import VLNEvaluator
from vln_evaluator.env import O3DEEnv
from vln_evaluator.policy import WebSocketPolicy

# 创建环境
env = O3DEEnv(simulator_config={
    'mode': 'socket',
    'success_threshold': 0.2
})

# 创建策略
policy = WebSocketPolicy('localhost', '8080')

# 创建评测器
with VLNEvaluator(env=env, policy=policy,
                   default_max_steps=50,
                   default_success_threshold=0.2) as evaluator:

    # 评测单个episode
    result = evaluator.evaluate_episode(r2r_episode)
    print(f"Success: {result.success}")
    print(f"Distance: {result.final_distance_to_goal:.3f}m")

    # 或评测整个数据集
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
    evaluator.save_results(results, 'results/eval_results.json')
```

## 配置说明

配置文件 `configs/vln_eval.yaml`:

```yaml
# O3DE Simulator配置
simulator:
  mode: socket                      # 'peb' 或 'socket'
  success_threshold: 0.2            # 成功距离阈值(米)
  collision_threshold: 0.3          # 碰撞检测阈值(米)
  linear_speed: 0.1                 # 线速度(m/s)
  angular_speed: 0.3                # 角速度(rad/s)

# VLM配置
vlm:
  url: ws://localhost:8080          # WebSocket URL

# 评测配置
evaluation:
  default_max_steps: 50             # 默认最大步数
  default_success_threshold: 0.2    # 默认成功阈值

# 日志配置
logging:
  level: INFO
```

## 输出结果

评测完成后，结果保存在JSON文件中：

```json
{
  "summary": {
    "total_episodes": 100,
    "success_count": 75,
    "success_rate": 0.75,
    "avg_distance_error": 0.15,
    "avg_steps": 25.3,
    "avg_collision_count": 1.2,
    "timeout_count": 5
  },
  "episodes": [
    {
      "episode_id": "ep_001",
      "success": true,
      "final_distance_to_goal": 0.12,
      "steps": 18,
      "collision_count": 2,
      "trajectory": [...]
    }
  ]
}
```

## 测试（使用Mock组件）

在不需要真实O3DE和VLM服务器的情况下，可以使用Mock组件测试评测流程：

### 安装测试环境依赖

```bash
# 创建conda环境（如果还没有）
conda create -n cloudrobo-nav python=3.10 -y
conda activate cloudrobo-nav

# 安装依赖
pip install websocket-client websocket-server pyyaml numpy
```

### 运行测试套件

```bash
python tests/test_evaluation.py
```

测试套件包含6个测试：

1. **Test 1: MockEnv** - 测试环境接口（reset/step/close）
2. **Test 2: MockPolicy** - 测试策略接口（act）
3. **Test 3: VLNEvaluator with Mocks** - 测试完整评测流程（MockEnv + MockPolicy）
4. **Test 4: VLNEvaluator with WebSocket** - 测试WebSocketPolicy
5. **Test 5: evaluate_dataset()** - 测试批量评测
6. **Test 6: Dataset Loader** - 测试R2R格式数据集

### 测试输出示例

```
======================================================================
VLN EVALUATION SYSTEM - TEST SUITE
======================================================================

✓ Test 1 PASSED - MockEnv
✓ Test 2 PASSED - MockPolicy
✓ Test 3 PASSED - VLNEvaluator (MockEnv + MockPolicy)
✓ Test 4 PASSED - VLNEvaluator (MockEnv + WebSocketPolicy)
✓ Test 5 PASSED - evaluate_dataset()
✓ Test 6 PASSED - Dataset Loader

Results: 6/6 tests passed
✓ ALL TESTS PASSED!
```

## 常见问题

### Q1: 提示找不到o3de_sim模块

确保O3DE的Python路径已添加到PYTHONPATH：

```bash
# 查找O3DE Python路径
find /path/to/o3de -name "o3de_sim" -type d

# 添加到环境变量
export PYTHONPATH=/path/to/o3de/python:$PYTHONPATH
```

### Q2: ROS2话题没有数据

检查以下几点：
1. O3DE是否进入Game Mode
2. ROS2 Frame组件是否正确配置
3. 检查话题QoS配置

```bash
# 查看话题详情
ros2 topic info /rgb -v

# 检查是否有数据
ros2 topic hz /rgb
```

### Q3: VLM连接失败

1. 确认VLM服务器是否运行：`netstat -an | grep 8080`
2. 检查防火墙设置
3. 尝试使用telnet测试：`telnet localhost 8080`

## 进阶使用

### 创建自定义Policy

```python
from vln_evaluator.policy import Policy

class MyCustomPolicy(Policy):
    def act(self, obs):
        rgb = obs['rgb']
        depth = obs['depth']
        instruction = obs['instruction']

        # 你的自定义逻辑
        action = my_model.predict(rgb, depth, instruction)

        return action  # 返回 0/1/2/3

# 使用
from vln_evaluator import VLNEvaluator, MockEnv

env = MockEnv()
policy = MyCustomPolicy()

with VLNEvaluator(env=env, policy=policy) as evaluator:
    results = evaluator.evaluate_dataset('datasets/sample_r2r.json')
```

### 创建自定义数据集

```python
from vln_evaluator.dataset_loader import R2REpisode, R2RDatasetLoader

episodes = [
    R2REpisode(
        episode_id='custom_001',
        scene_id='my_scene',
        instruction='Navigate to the red door',
        start_position={'x': 0, 'y': 0, 'z': 0},
        start_rotation={'x': 0, 'y': 0, 'z': 0},
        goal_position={'x': 5, 'y': 0, 'z': 2},
        max_steps=100
    )
]

R2RDatasetLoader.save_to_file(episodes, 'datasets/my_dataset.json')
```

## 文档

- [VLN评测系统设计](docs/VLN_EVALUATION_DESIGN.md) - 需求分析和架构设计
- [O3DE Simulator设计](docs/O3DE_SIMULATOR_DESIGN.md) - 仿真器接口设计
- [实现细节](docs/IMPLEMENTATION_DETAILS.md) - 代码实现细节和扩展指南

## 许可证

[待添加]

## 联系方式

[待添加]
