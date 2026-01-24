# VLN Evaluation System - O3DE

基于O3DE仿真环境的视觉-语言导航（VLN）评测系统。

## 项目简介

本系统提供完整的VLN评测流程，支持：
- **O3DE仿真环境**：基于Open 3D Engine的物理仿真
- **ROS2集成**：通过ROS2话题获取传感器数据和控制机器人
- **远程VLM**：通过WebSocket连接远程视觉-语言模型
- **R2R数据集**：支持R2R格式的导航指令数据集
- **Habitat风格接口**：reset/step/close标准接口

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
├── vln_evaluator/                  # 评测器模块
│   ├── __init__.py
│   ├── dataset_loader.py           # R2R数据集加载器
│   ├── ws_policy_client.py         # WebSocket VLM客户端
│   └── vln_evaluator.py            # 主评测器
├── o3de_sim/                       # O3DE仿真接口库
│   └── o3de_api/
├── o3de_simulator.py               # O3DE仿真器
├── action_executor.py              # ROS2动作执行器
├── policy.py                       # WebSocket Policy
├── run_vln_eval.py                 # 评测运行脚本
├── configs/
│   └── vln_eval.yaml               # 配置文件
├── datasets/
│   └── sample_r2r.json            # 示例数据集
└── results/                        # 结果输出目录
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

在终端中运行：

```bash
# 检查话题列表
ros2 topic list

# 应该看到以下话题：
# /rgb
# /depth
# /scan
# /cmd_vel
# /odom

# 检查话题数据
ros2 topic hz /rgb
ros2 topic echo /rgb --once
```

### 步骤3: 启动VLM服务器

根据你的VLM模型启动WebSocket服务器。服务器需要：

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

示例VLM服务器（伪代码）:
```python
import websocket
import pickle
import numpy as np

def on_message(ws, message):
    obs = pickle.loads(message)
    rgb = obs['rgb']
    depth = obs['depth']
    instruction = obs['instruction']

    # 调用你的VLM模型
    action = your_vlm_model(rgb, depth, instruction)

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

或使用提供的示例数据集：

```bash
cp datasets/sample_r2r.json my_dataset.json
# 编辑 my_dataset.json 添加你的episodes
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

# 指定配置文件
python run_vln_eval.py \
    --dataset datasets/sample_r2r.json \
    --output results/eval_results.json \
    --config configs/vln_eval.yaml

# 覆盖配置项
python run_vln_eval.py \
    --dataset datasets/sample_r2r.json \
    --output results/eval_results.json \
    --sim-mode socket \
    --vlm-url ws://localhost:9999
```

### 方式2: 使用Python API

```python
from vln_evaluator import VLNEvaluator, evaluate_vln

# 使用便捷函数
evaluate_vln(
    dataset_path='datasets/sample_r2r.json',
    output_path='results/eval_results.json',
    simulator_config={
        'mode': 'socket',
        'success_threshold': 0.2,
        'linear_speed': 0.1,
        'angular_speed': 0.3
    },
    vlm_config={
        'url': 'ws://localhost:8080',
        'use_pickle': True
    }
)

# 或使用类（更灵活控制）
from vln_evaluator import VLNEvaluator

with VLNEvaluator(
    simulator_config={'mode': 'socket'},
    vlm_config={'url': 'ws://localhost:8080'}
) as evaluator:

    # 运行评测
    results = evaluator.evaluate_dataset(
        dataset_path='datasets/sample_r2r.json'
    )

    # 保存结果
    evaluator.save_results(results, 'results/eval_results.json')

    # 访问统计
    print(f"Success Rate: {results.success_rate:.2%}")
    print(f"Avg Distance: {results.avg_distance_error:.3f}m")
```

### 方式3: 直接使用Simulator

```python
from o3de_simulator import O3DESimulator, Episode

# 创建仿真器
sim = O3DESimulator(mode='socket', success_threshold=0.2)

# 定义episode
episode = Episode(
    episode_id='ep_001',
    scene_id='scene_001',
    start_position={'x': 0, 'y': 0, 'z': 0},
    start_rotation={'x': 0, 'y': 0, 'z': 0},
    goal_position={'x': 3, 'y': 0, 'z': 0},
    instruction='Walk forward 3 meters',
    max_steps=50
)

# Reset
obs = sim.reset(episode)

# 评测循环
for step in range(50):
    # 获取action（这里需要你自己调用VLM）
    action = your_vlm_function(obs['rgb'], obs['depth'], episode.instruction)

    # Step
    obs, reward, done, info = sim.step(action)

    print(f"Step {step}: distance={info['distance_to_goal']:.3f}m")

    if done:
        print(f"Done! Success: {info['distance_to_goal'] < 0.2}")
        break

sim.close()
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
  socket_host: 127.0.0.1
  socket_port: 8080

# VLM配置
vlm:
  url: ws://localhost:8080          # WebSocket URL
  use_pickle: true                  # 序列化方式（true=pickle, false=json）

# 评测配置
evaluation:
  default_max_steps: 50             # 默认最大步数
  default_success_threshold: 0.2    # 默认成功阈值
  output_dir: results
  save_trajectories: true

# 日志配置
logging:
  level: INFO
  log_file: logs/vln_eval.log
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
      "scene_id": "scene_001",
      "instruction": "Walk straight forward...",
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

## 常见问题

### Q1: 提示找不到o3de_sim模块

**A**: 确保O3DE的Python路径已添加到PYTHONPATH：

```bash
# 查找O3DE Python路径
find /path/to/o3de -name "o3de_sim" -type d

# 添加到环境变量
export PYTHONPATH=/path/to/o3de/python:$PYTHONPATH
```

### Q2: ROS2话题没有数据

**A**: 检查以下几点：
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

**A**:
1. 确认VLM服务器是否运行：`netstat -an | grep 8080`
2. 检查防火墙设置
3. 尝试使用telnet测试：`telnet localhost 8080`

### Q4: 机器人不动

**A**:
1. 检查O3DE是否启用了物理模拟
2. 确认机器人有刚体组件
3. 检查/cmd_vel话题：`ros2 topic echo /cmd_vel`

### Q5: 碰撞检测不工作

**A**:
1. 确认O3DE场景中有LaserScan传感器
2. 检查/scan话题：`ros2 topic echo /scan`
3. 调整碰撞阈值（configs/vln_eval.yaml中的collision_threshold）

## 命令参考

### 完整命令列表

```bash
python run_vln_eval.py [OPTIONS]

选项:
  --dataset PATH        R2R数据集JSON文件路径
  --output PATH         结果输出文件路径
  --config PATH         配置文件路径 (默认: configs/vln_eval.yaml)
  --episodes EPISODES   要评测的episode ID列表 (可选)
  --vlm-url URL         VLM WebSocket服务器URL (覆盖配置文件)
  --sim-mode MODE       O3DE模式: peb或socket (覆盖配置文件)
  -h, --help            显示帮助信息
```

### ROS2调试命令

```bash
# 列出所有话题
ros2 topic list

# 查看话题类型和QoS
ros2 topic info /rgb -v

# 查看话题数据
ros2 topic echo /rgb

# 查看话题频率
ros2 topic hz /rgb

# 录制话题数据
ros2 bag record /rgb /depth /scan

# 回放bag文件
ros2 bag play recorded_bag
```

## 进阶使用

### 创建自定义数据集

```python
from vln_evaluator.dataset_loader import R2REpisode, R2RDatasetLoader

# 创建episodes
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

# 保存数据集
R2RDatasetLoader.save_to_file(episodes, 'datasets/my_dataset.json')
```

### 扩展评测指标

编辑 `vln_evaluator/vln_evaluator.py`，添加自定义指标计算：

```python
# 在EvaluationResult中添加新字段
@dataclass
class EvaluationResult:
    # 现有字段...
    custom_metric: float = 0.0

# 在evaluate_dataset中计算
for ep_result in results.episodes:
    ep_result.custom_metric = calculate_custom(ep_result)
```

### 可视化轨迹

```python
import matplotlib.pyplot as plt

# 读取结果
import json
with open('results/eval_results.json') as f:
    results = json.load(f)

# 绘制第一个episode的轨迹
traj = results['episodes'][0]['trajectory']
x = [p['x'] for p in traj]
z = [p['z'] for p in traj]

plt.plot(x, z, '-o')
plt.xlabel('X (m)')
plt.ylabel('Z (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.axis('equal')
plt.savefig('trajectory.png')
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
# 运行完整测试套件
python tests/test_evaluation.py
```

测试套件包含4个测试：

1. **Test 1: Mock O3DE Simulator** - 测试模拟仿真器的reset/step/close接口
2. **Test 2: VLM Client** - 测试WebSocket客户端与Mock VLM服务器的通信
3. **Test 3: 完整评测流程** - 测试Mock Simulator + Mock VLM的完整评测流程
4. **Test 4: 数据集加载器** - 测试R2R格式数据集的加载和保存

### 测试输出示例

```
======================================================================
VLN EVALUATION SYSTEM - TEST SUITE
======================================================================

✓ Test 1 PASSED - Mock O3DE Simulator (No VLM)
✓ Test 2 PASSED - VLM Client (Mock Server Required)
✓ Test 3 PASSED - Full Evaluation (Mock Simulator + Mock VLM)
✓ Test 4 PASSED - R2R Dataset Loader

Results: 4/4 tests passed
✓ ALL TESTS PASSED!
```

### 单独测试Mock组件

#### 测试Mock Simulator

```python
from tests.mock_o3de_simulator import MockO3DESimulator, MockEpisode

sim = MockO3DESimulator(success_threshold=0.2)

episode = MockEpisode(
    episode_id='test_001',
    start_position={'x': 0, 'y': 0, 'z': 0},
    goal_position={'x': 1.0, 'y': 0, 'z': 0},
    instruction='Walk forward 1 meter',
    max_steps=10
)

# Reset and run
obs = sim.reset(episode)
for step in range(5):
    obs, reward, done, info = sim.step(1)  # FORWARD
    print(f"Step {step+1}: {info}")

sim.close()
```

#### 启动Mock VLM Server

```bash
# 启动Mock VLM服务器（普通模式）
python tests/mock_vlm_server.py --port 8080

# 启动Mock VLM服务器（智能导航模式）
python tests/mock_vlm_server.py --port 8080 --smart

# 使用JSON序列化
python tests/mock_vlm_server.py --port 8080 --json
```

#### 测试WebSocket通信

```python
from vln_evaluator.ws_policy_client import WebSocketPolicyClient
import numpy as np

# 连接Mock服务器
client = WebSocketPolicyClient(url='ws://localhost:8080')
client.connect()

# 测试通信
rgb = np.random.randint(0, 255, (64, 64, 3), dtype=np.uint8)
depth = np.random.randint(0, 10000, (64, 64), dtype=np.uint16)
instruction = "Walk forward 2 meters"

action = client.act(rgb, depth, instruction)
print(f"Received action: {action}")  # 0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT

client.disconnect()
```

### Mock组件说明

**MockO3DESimulator** (`tests/mock_o3de_simulator.py`):
- 模拟O3DE仿真器，无需真实的O3DE引擎
- 模拟机器人运动、传感器数据（RGB/Depth）、碰撞检测
- 提供与真实O3DESimulator相同的接口

**Mock VLM Server** (`tests/mock_vlm_server.py`):
- WebSocket服务器，模拟VLM模型行为
- 支持两种模式：随机模式、智能导航模式
- 支持pickle和JSON序列化

## 文档

- [VLN评测系统设计](docs/VLN_EVALUATION_DESIGN.md) - 需求分析和架构设计
- [O3DE Simulator设计](docs/O3DE_SIMULATOR_DESIGN.md) - 仿真器接口设计
- [实现细节](docs/IMPLEMENTATION_DETAILS.md) - 代码实现细节和扩展指南

## 许可证

[待添加]

## 联系方式

[待添加]
