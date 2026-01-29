# Habitat-Sim 离散动作实现分析

## 一、概述

本文档分析 **habitat-sim** 如何实现 VL N 离散动作（前进 25cm、旋转 15°），特别关注：
1. **动作定义与注册机制**
2. **距离控制实现**
3. **碰撞检测机制**
4. **与当前 action_executor.py 的对比**

---

## 二、动作定义与注册机制

### 2.1 动作空间（Action Space）

Habitat-sim 使用 `ActionSpec` 定义动作：

```python
from habitat_sim.agent import ActuationSpec

# 定义动作
agent_config.action_space = {
    "move_forward": ActionSpec(
        "move_forward",                          # 动作名称（注册名）
        ActuationSpec(amount=0.25)              # 动作参数：前进距离（米）
    ),
    "turn_left": ActionSpec(
        "turn_left",
        ActuationSpec(amount=30.0)           # 旋转角度（度）
    ),
    "turn_right": ActionSpec(
        "turn_right",
        ActuationSpec(amount=30.0)
    ),
}
```

**关键点**：
- **动作名称**：`"move_forward"`, `"turn_left"`, `"turn_right"`
- **参数**：`ActuationSpec(amount=...)` - 可自定义前进距离和旋转角度
- **默认值**：前进 `0.25` 米（25cm），旋转 `30.0` 度

---

### 2.2 动作注册机制（Decorator 模式）

Habitat-sim 使用 **registry.register_move_fn()** 装饰器注册动作：

```python
from habitat_sim.registry import registry
from habitat_sim.agent.controls.controls import SceneNodeControl

@registry.register_move_fn(body_action=True)
class MoveForward(SceneNodeControl):
    """
    注册为 body_action=True，表示这个动作会直接影响智能体的刚体

    body_action=True：动作会修改智能体的物理状态（位置、旋转）
    body_action=False：动作只影响传感器（如相机）
    """

    def __call__(
        self,
        scene_node: habitat_sim.SceneNode,     # 场景节点（智能体）
        actuation_spec: ActuationSpec,        # 动作参数（如 amount=0.25）
    ) -> None:
        # 动作实现
        pass
```

**注册特点**：
1. **装饰器模式**：`@registry.register_move_fn()` 自动注册
2. **动作分类**：`body_action` 控制物理，非 body 控制传感器
3. **动态查找**：通过动作名称查找对应的控制函数
4. **可重写**：用户可以在核心库外部定义自己的动作

---

## 三、距离控制实现

### 3.1 默认动作：MoveForward（前进 0.25m）

#### 3.1.1 核心实现

```python
from habitat_sim.geo import FRONT

@registry.register_move_fn(body_action=True)
class MoveForward(SceneNodeControl):
    def __call__(
        self,
        scene_node: habitat_sim.SceneNode,
        actuation_spec: ActuationSpec
    ) -> None:
        """
        沿着 +Z 轴（前方）移动指定距离

        Args:
            scene_node: 智能体的场景节点
            actuation_spec: ActuationSpec(amount=0.25) 表示移动 0.25 米
        """
        # 获取前方方向向量
        forward_ax = (
            np.array(scene_node.absolute_transformation().rotation_scaling())
            @ FRONT
        )

        # 执行本地平移（相对于智能体当前朝向）
        scene_node.translate_local(forward_ax * actuation_spec.amount)
```

**关键点**：
- **移动方向**：`FRONT`（+Z 轴）- 假设智能体默认面向 +Z
- **坐标系**：使用 `translate_local()` - 相对于智能体的局部坐标系
- **移动距离**：`actuation_spec.amount` - 默认 0.25 米
- **一步到位**：动作是一次性移动，没有时间积分

---

### 3.2 旋转动作：TurnLeft / TurnRight（旋转 30°）

```python
@registry.register_move_fn(body_action=True)
class TurnLeft(SceneNodeControl):
    def __call__(
        self,
        scene_node: habitat_sim.SceneNode,
        actuation_spec: ActuationSpec
    ) -> None:
        """绕 +Y 轴（上方）旋转指定角度"""
        rotation_ax = mn.Deg(actuation_spec.amount)
        scene_node.rotate_local(rotation_ax, rotation_ax)
```

**关键点**：
- **旋转轴**：`UP`（+Y 轴）
- **旋转方式**：`rotate_local()` - 局部旋转
- **角度单位**：度（`mn.Deg(...)`）
- **归一化**：`scene_node.rotation = scene_node.rotation.normalized()`

---

### 3.3 与当前 action_executor.py 的对比

| 特性 | Habitat-Sim | 当前 action_executor.py | 差异 |
|------|-------------|----------------------|------|
| **移动方式** | `translate_local()` 一次性到位 | `/cmd_vel` + 时间积分 | 仿真方式不同 |
| **距离控制** | `ActuationSpec(amount=0.25)` | `odom` 闭环控制 | Habitat：预设值，O3DE：实时反馈 |
| **旋转控制** | `rotate_local()` 一次性旋转 | `odom` 角度闭环 | Habitat：预设值，O3DE：实时反馈 |
| **动作粒度** | 粗粒度（固定 25cm/30°） | 粗粒度（固定 25cm/15°） | 相同 |
| **物理效果** | 瞬间物理引擎步进 | 持续物理模拟 | 不同 |

**优势对比**：

**Habitat-Sim 的优势**：
- ✅ **简单**：一次性动作，无需循环控制
- ✅ **高效**：不依赖实时传感器反馈
- ✅ **可预测**：动作参数固定，行为确定

**O3DE action_executor.py 的优势**：
- ✅ **精确**：基于 odom 的闭环控制，适应物理波动
- ✅ **灵活**：可根据传感器数据实时调整速度
- ✅ **仿真真实性**：符合真实机器人的控制方式

---

## 四、碰撞检测机制

### 4.1 碰撞检测架构

Habitat-sim 提供两层碰撞检测：

#### 4.1.1 Agent.act() 方法（主要检测）

```python
# habitat_sim/src_python/habitat_sim/agent/agent.py

def act(self, action_id: Any) -> bool:
    """
    执行动作，返回是否发生碰撞

    Returns:
        bool: True = 碰撞，False = 无碰撞
    """
    action = self.agent_config.action_space[action_id]
    did_collide = False

    if self.controls.is_body_action(action.name):
        # body_action：直接修改智能体物理状态
        did_collide = self.controls.action(
            self.scene_node,
            action.name,
            action.actuation,
            apply_filter=True  # 启用碰撞过滤
        )
    else:
        # 非 body_action：移动传感器（如相机）
        for v in self._sensors.values():
            self.controls.action(
                v.object,
                action.name,
                action.actuation,
                apply_filter=False  # 不应用过滤
            )

    return did_collide
```

**关键点**：
- **返回值**：`bool` - 是否碰撞
- **分类处理**：`body_action` 和非 `body_action` 区分处理
- **碰撞过滤**：`apply_filter=True/False`

---

#### 4.1.2 碰撞过滤：ObjectControls.action()

```python
# habitat_sim/src_python/habitat_sim/agent/controls/object_controls.py

def action(
    self,
    obj: hsim.SceneNode,
    action_name: str,
    actuation_spec: ActuationSpec,
    apply_filter: bool = True,      # 是否应用碰撞过滤
) -> bool:
    """
    执行动作并检测碰撞

    核心机制：
    1. 记录移动前位置
    2. 执行移动
    3. 应用 move_filter_fn
    4. 比较移动距离检测碰撞
    """
    start_pos = obj.absolute_translation

    # 执行移动
    move_fn = registry.get_move_fn(action_name)
    move_fn(obj, actuation_spec)

    end_pos = obj.absolute_translation

    collided = False

    if apply_filter:
        # 应用移动过滤函数
        filter_end = self.move_filter_fn(start_pos, end_pos)

        # 更新位置以尊重过滤器
        obj.translate(filter_end - end_pos)

        # 比较移动距离
        dist_moved_before_filter = (end_pos - start_pos).dot()
        dist_moved_after_filter = (filter_end - start_pos).dot()

        # 碰撞判定：移动距离小于阈值
        EPS = 1e-5  # 机器精度容差
        collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

    return collided
```

**核心算法**：
```
1. start_pos = 记录移动前位置
2. 执行移动 → 移动物理引擎计算新位置
3. end_pos = 获取移动后位置
4. filter_end = move_filter_fn(start_pos, end_pos)  # 可选：应用导航约束
5. dist_before = |end_pos - start_pos|（实际移动距离）
6. dist_after = |filter_end - start_pos|（允许移动距离）
7. 碰撞判定：dist_after + EPS < dist_before
```

**优点**：
- ✅ **简单高效**：只需比较移动距离
- ✅ **支持导航过滤**：可集成 Recast Navigation 等导航网格
- ✅ **可配置**：容差 `EPS` 可调整

---

### 4.2 导航网格过滤（可选）

```python
# simulator.py

def step_filter(self, start_pos: Vector3, end_pos: Vector3) -> Vector3:
    """
    通过导航网格过滤目标位置

    如果启用导航网格，使用 try_step() 应用导航约束
    否则直接返回目标位置
    """
    if self.pathfinder.is_loaded:
        if self.config.sim_cfg.allow_sliding:
            # 允许沿障碍物滑动
            end_pos = self.pathfinder.try_step(start_pos, end_pos)
        else:
            # 不允许滑动，强制导航网格约束
            end_pos = self.pathfinder.try_step_no_sliding(start_pos, end_pos)
    else:
        # 无导航网格，直接返回
        end_pos = end_pos

    return end_pos
```

**关键点**：
- **try_step()**：允许智能体沿障碍物滑动（更真实）
- **try_step_no_sliding()**：强制导航网格约束（更安全）
- **滑动标志**：`allow_sliding` 配置项

---

### 4.3 碰撞检测调用流程

```python
# simulator.py - step() 方法

def step(self, action: Union[str, int], dt: float = 1.0/60.0) -> Dict[int, ObservationDict]:
    """
    步进仿真

    Returns:
        observations: 包含 "collided" 字段
    """
    collided_dict: Dict[int, bool] = {}

    # 1. 执行所有智能体的动作
    for agent_id, agent_act in action.items():
        agent = self.get_agent(agent_id)
        collided_dict[agent_id] = agent.act(agent_act)

        # 2. 物理步进（dt 秒）
    self.step_world(dt)

    # 3. 收集传感器观察
    multi_observations = self.get_sensor_observations(agent_ids=list(action.keys()))

    # 4. 添加碰撞信息到观察
    for agent_id, agent_observation in multi_observations.items():
        agent_observation["collided"] = collided_dict[agent_id]

    return multi_observations
```

**数据流**：
```
action → agent.act() → 返回 collided
        ↓
simulator.step() → 碰撞信息添加到 observations
        ↓
observations["collided"] = True/False
```

---

## 五、与当前实现的关键差异

### 5.1 动作执行方式

| 方面 | Habitat-Sim | action_executor.py | 对 VL N 的影响 |
|------|-------------|----------------------|--------------|
| **动作定义** | `ActionSpec(amount=0.25)` | `/cmd_vel` + 时间积分 | Habitat：确定性，O3DE：更真实 |
| **执行方式** | `translate_local()` 一次性到位 | 速度控制 + odom 反馈 | Habitat：简单但不够精确 |
| **时间控制** | 由动作参数决定 | 由仿真器步进决定 | Habitat：动作粒度粗，O3DE：精确可控 |
| **碰撞检测** | 移动距离比较 | LaserScan 距离检测 | 不同原理 |

### 5.2 碰撞检测原理

| 特性 | Habitat-Sim | action_executor.py |
|------|-------------|------------------|
| **检测时机** | 动作执行后立即检测 | 持续监听 `/scan` 话题 |
| **检测原理** | 比较实际移动距离 vs 允许距离 | 前方最小距离 < 阈值 |
| **导航约束** | 支持 Recast Navigation 过滤 | 不支持（需要手动实现） |
| **灵活性** | 高（可自定义 move_filter_fn） | 中（固定逻辑） |

### 5.3 实时性 vs. 预测性

**Habitat-Sim（预预测性）**：
```python
# 动作参数固定
ActionSpec(amount=0.25)  # 每次前进 25cm
ActuationSpec(amount=30.0)  # 每次旋转 30°

# 执行后立即完成
agent.act("move_forward")  # 返回 collided bool
```

**action_executor.py（实时间应性）**：
```python
# 动作参数固定，但实际位移由物理引擎决定
self.LINEAR_SPEED = 0.1  # m/s
execute_forward_25cm():
    while not collision_detected:
        self.cmd_vel.publish(Twist(linear.x=0.1))  # 持续发布
        if distance_traveled >= 0.25:
            break
```

---

## 六、对 action_executor.py 的优化建议

### 6.1 引入动作配置

**借鉴 Habitat-Sim 的 `ActionSpec`**：

```python
class VLNActionExecutor(Node):
    def __init__(self):
        # ...

        # === 添加动作配置 ===
        self.action_configs = {
            "FORWARD": {"distance": 0.25, "speed": 0.1},
            "LEFT": {"angle": 15.0, "speed": 0.3},
            "RIGHT": {"angle": 15.0, "speed": 0.3},
        }

        # 当前动作参数（可动态调整）
        self.current_action = None

    def configure_action(self, action_id: int, **params):
        """动态配置动作参数"""
        if action_id == 1:  # FORWARD
            self.action_configs["FORWARD"].update(params)
        elif action_id == 2:  # LEFT
            self.action_configs["LEFT"].update(params)
        elif action_id == 3:  # RIGHT
            self.action_configs["RIGHT"].update(params)
```

### 6.2 引入碰撞过滤

**借鉴 Habitat-Sim 的 `apply_filter`**：

```python
from typing import Callable, Tuple, Union

class VLNActionExecutor(Node):
    def __init__(self):
        # ...

        # === 碰撞过滤器 ===
        # 可用于集成 Recast Navigation 或其他约束
        self.move_filter_fn: Callable[
            Tuple[np.ndarray, np.ndarray],  # (start_pos, end_pos)
            np.ndarray                              # filtered_end_pos
        ] = lambda start, end: end  # 默认不过滤

    def apply_collision_filter(self, start_pos: np.ndarray, end_pos: np.ndarray):
        """应用碰撞过滤（如果已配置）"""
        if self.move_filter_fn is not None:
            filtered_end = self.move_filter_fn(start_pos, end_pos)
            # 计算实际允许移动距离
            actual_dist = np.linalg.norm(filtered_end - start)
            filtered_dist = np.linalg.norm(filtered_end - start)

            # 容差判断（Habitat-Sim 方式）
            EPS = 1e-5
            collided = (filtered_dist + EPS) < actual_dist

            return collided, filtered_end

        # 未配置过滤器，直接返回
        return False, end_pos
```

### 6.3 改进动作执行逻辑

**结合 Habitat-Sim 的一次性动作和 O3DE 的实时控制**：

```python
def execute_forward_25cm_enhanced(self):
    """
    增强版前进动作：
    - 记录起始位置
    - 执行移动（通过碰撞过滤）
    - 检测碰撞
    """
    if not self.wait_for_odom():
        return

    # 记录起始位置（应用过滤器）
    start_pos_before = np.array([
        self.curr_pose.position.x,
        self.curr_pose.position.y,
        self.curr_pose.position.z
    ])

    self.is_running = True
    self.collision_detected = False

    while rclpy.ok():
        rclpy.spin_once(self, timeout_sec=0)

        # === 应用碰撞过滤 ===
        did_filter_collide, filtered_end_pos = self.apply_collision_filter(
            start_pos_before,
            np.array([
                self.curr_pose.position.x,
                self.curr_pose.position.y,
                self.curr_pose.position.z
            ])
        )

        if did_filter_collide:
            # 过滤器检测到碰撞，立即停止
            self.collision_detected = True
            self.collision_count += 1
            print(f"碰撞过滤检测到碰撞！总次数: {self.collision_count}")
            break

        # === 计算实际移动距离 ===
        dist = math.sqrt(
            (self.curr_pose.position.x - start_pos_before[0]) ** 2 +
            (self.curr_pose.position.y - start_pos_before[1]) ** 2
        )

        # === 终止条件 ===
        if dist >= 0.25 or self.collision_detected:
            break

        # === 发布速度指令 ===
        msg = Twist()
        msg.linear.x = self.LINEAR_SPEED
        self.cmd_pub.publish(msg)
        time.sleep(0.05)

    self.stop_robot()
```

**优点**：
- ✅ **双重检测**：LaserScan + 碰撞过滤器
- ✅ **支持导航过滤**：可集成 Recast Navigation
- ✅ **更精确**：结合预计算和实时检测
- ✅ **更安全**：多层碰撞检测

---

## 七、集成 Recast Navigation 的方案

### 7.1 添加导航网格支持

**参考 Habitat-Sim 的 `step_filter()`**：

```python
class O3DESimulator:
    def __init__(self):
        # ...

        # === Recast Navigation ===
        self.recast_nav_loaded = False
        self.pathfinder = None  # O3DE PathFinder

    def load_recast_navigation(self, scene_config_path: str):
        """加载 Recast Navigation Mesh"""
        from o3de_sim.o3de_api.peb_extras import recast_navigation_tool

        # 生成导航网格
        recast_navigation_tool.update_navigation_mesh_sync(
            self.recast_nav_entity_id
        )

        # 获取 PathFinder
        self.pathfinder = recast_navigation_tool.get_pathfinder()
        self.recast_nav_loaded = True

    def step_filter(self, start_pos: np.ndarray, end_pos: np.ndarray) -> np.ndarray:
        """通过导航网格过滤目标位置"""
        if not self.recast_nav_loaded or self.pathfinder is None:
            # 未加载导航网格，直接返回
            return end_pos

        # 使用 try_step 应用导航约束
        filtered_pos = self.pathfinder.try_step(
            start_pos,
            end_pos
        )

        return filtered_pos
```

### 7.2 在 action_executor 中集成

```python
class VLNActionExecutor(Node):
    def __init__(self):
        # ...

        # === 导航网格过滤器（来自 O3DESimulator）===
        self.nav_filter_fn: Optional[
            Callable[
                Tuple[np.ndarray, np.ndarray],
                np.ndarray
            ]
        ] = None

    def set_navigation_filter(self, filter_fn):
        """设置导航网格过滤器"""
        self.nav_filter_fn = filter_fn

    def execute_forward_with_nav_filter(self, distance: float = 0.25):
        """使用导航过滤的前进动作"""
        if not self.wait_for_odom():
            return

        start_pos = np.array([
            self.curr_pose.position.x,
            self.curr_pose.position.y,
            self.curr_pose.position.z
        ])

        self.is_running = True
        self.collision_detected = False

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)

            # 1. 应用导航过滤器（检查目标是否可达）
            if self.nav_filter_fn is not None:
                current_pos = np.array([
                    self.curr_pose.position.x,
                    self.curr_pose.position.y,
                    self.curr_pose.position.z
                ])
                # 目标位置（基于当前朝向和距离）
                target_pos = self.calculate_target_pos(distance)

                # 应用过滤器
                _, filtered_target = self.nav_filter_fn(current_pos, target_pos)

                # 检查过滤后距离
                filtered_dist = np.linalg.norm(filtered_target - current_pos)
                if filtered_dist < distance * 0.5:
                    print("目标位置导航网格约束：距离不足")
                    break

            # 2. 应用碰撞过滤器
            did_collide, _ = self.apply_collision_filter(start_pos, current_pos)
            if did_collide:
                self.collision_detected = True
                self.collision_count += 1
                print(f"碰撞过滤检测到碰撞！")
                break

            # 3. 检查实际移动距离
            dist = math.sqrt(
                (self.curr_pose.position.x - start_pos[0]) ** 2 +
                (self.curr_pose.position.y - start_pos[1]) ** 2
            )

            if dist >= distance or self.collision_detected:
                break

            msg = Twist()
            msg.linear.x = self.LINEAR_SPEED
            self.cmd_vel.publish(msg)
            time.sleep(0.05)

        self.stop_robot()

    def calculate_target_pos(self, distance: float) -> np.ndarray:
        """基于当前朝向和距离计算目标位置"""
        # 获取前方方向
        q = self.curr_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 前方方向（假设 +X 轴为前方）
        target_x = self.curr_pose.position.x + distance * math.cos(yaw)
        target_y = self.curr_pose.position.y + distance * math.sin(yaw)
        target_z = self.curr_pose.position.z

        return np.array([target_x, target_y, target_z])
```

---

## 八、总结与建议

### 8.1 Habitat-Sim 的关键优势

| 优势 | 说明 |
|------|------|
| **架构清晰** | 动作定义（ActionSpec）+ 注册（register_move_fn）+ 执行（control）分离 |
| **灵活性强** | 支持自定义动作、参数可配、可扩展 |
| **多层碰撞检测** | agent.act() 主检测 + apply_filter 辅助过滤 |
| **导航集成** | 支持 Recast Navigation 网格约束 |
| **简单高效** | 一次性动作，无需循环控制 |

### 8.2 对 action_executor.py 的改进建议

#### 短期优化（立即可用）

1. **添加动作配置类**
   ```python
   class ActionConfig:
       distance: float
       speed: float
       collision_threshold: float
   ```

2. **分离碰撞检测和动作执行**
   - 将碰撞检测逻辑独立为 CollisionDetector 类
   - 保持动作执行器专注于运动控制

3. **添加碰撞过滤器机制**
   - 参考 `ObjectControls.action()` 的 `apply_filter` 逻辑
   - 支持自定义过滤器（导航网格、安全区域等）

#### 中期优化（需要开发）

1. **集成 Recast Navigation**
   - 在 O3DE 中加载导航网格
   - 实现 `step_filter()` 应用导航约束
   - 避免机器人移动到不可达区域

2. **改进距离控制精度**
   - 使用碰撞过滤器提高精度
   - 减少物理引擎波动的影响

#### 长期优化（可选）

1. **动作学习**
   - 学习最优的速度参数
   - 根据环境自适应调整动作参数
   - 减少碰撞和导航时间

---

## 九、示例：改进后的动作执行器

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import math
import numpy as np
import time

class VLNActionExecutor(Node):
    def __init__(self):
        super().__init__('vln_action_executor')

        # === QoS 配置 ===
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # === 话题 ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, '/depth', self.depth_callback, sensor_qos)

        # === 状态变量 ===
        self.curr_pose = None
        self.collision_detected = False
        self.collision_count = 0
        self.is_running = False

        # === 参数配置 ===
        self.action_config = {
            "FORWARD": {"distance": 0.25, "speed": 0.1},
            "LEFT": {"angle": 15.0, "speed": 0.3},
            "RIGHT": {"angle": 15.0, "speed": 0.3},
        }

        # === 碰撞过滤器 ===
        self.collision_filter_fn = None
        self.nav_filter_fn = None

    def set_collision_filter(self, filter_fn):
        """设置碰撞过滤器（支持 Recast Navigation）"""
        self.collision_filter_fn = filter_fn

    def set_navigation_filter(self, filter_fn):
        """设置导航网格过滤器"""
        self.nav_filter_fn = filter_fn

    def odom_callback(self, msg):
        """里程计回调（保持原逻辑）"""
        header = msg.header
        pose = msg.pose.pose
        pos = pose.position

        if header.stamp.sec == 0:
            quat = pose.orientation
            is_zero_pose = (
                abs(pos.x) < 1e-6 and abs(pos.y) < 1e-6 and abs(pos.z) < 1e-6 and
                abs(quat.x) < 1e-6 and abs(quat.y) < 1e-6 and
                abs(quat.z) < 1e-6 and abs(quat.w - 1.0) < 1e-3
            )
            if is_zero_pose:
                return

        self.curr_pose = pose

    def scan_callback(self, msg):
        """激光雷达回调（保持原逻辑，但增加碰撞过滤器支持）"""
        if not self.is_running or self.collision_detected:
            return

        ranges = msg.ranges
        if not ranges:
            return

        num_points = len(ranges)
        requested_window = 30
        window = min(requested_window, num_points // 2)
        front_view = list(ranges[:window]) + list(ranges[-window:])
        valid_ranges = [r for r in front_view if msg.range_min < r < msg.range_max]

        if valid_ranges:
            current_min = min(valid_ranges)
            if current_min < self.COLLISION_THRESHOLD:
                # === 应用碰撞过滤器 ===
                if self.collision_filter_fn is not None:
                    # 使用自定义过滤器检测
                    pass
                else:
                    # 直接触发碰撞
                    if not self.collision_detected:
                        self.collision_detected = True
                        self.collision_count += 1
                        print(f"检测到碰撞！总次数: {self.collision_count}")

    def execute_forward_25cm(self):
        """前进动作（支持碰撞过滤）"""
        if not self.wait_for_odom():
            return

        distance = self.action_config["FORWARD"]["distance"]
        speed = self.action_config["FORWARD"]["speed"]

        self.get_logger().info(f"执行动作：前进 {distance}m（基于 odom）")
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

            # === 应用碰撞/导航过滤器 ===
            if self.collision_filter_fn is not None:
                start_pos = np.array([start_x, start_y, self.curr_pose.position.z])
                curr_pos = np.array([
                    self.curr_pose.position.x,
                    self.curr_pose.position.y,
                    self.curr_pose.position.z
                ])

                did_filter_collide, filtered_pos = self.collision_filter_fn(start_pos, curr_pos)
                if did_filter_collide:
                    self.collision_detected = True
                    self.collision_count += 1
                    print(f"过滤器检测到碰撞！总次数: {self.collision_count}")
                    break

            if self.collision_detected or dist >= distance:
                break

            msg = Twist()
            msg.linear.x = speed
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self.stop_robot()

    def execute_rotate_15deg(self, direction="left"):
        """旋转动作（保持原逻辑）"""
        if not self.wait_for_odom():
            return

        angle_deg = self.action_config[direction.upper()]["angle"]
        speed = self.action_config[direction.upper()]["speed"]

        self.get_logger().info(f"执行动作：{direction}转 {angle_deg}°（基于 odom）")
        start_yaw = self.get_yaw()
        target_yaw = start_yaw + math.radians(angle_deg) if direction == "left" else -math.radians(angle_deg)
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

        self.is_running = True
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            current_yaw = self.get_yaw()
            diff = target_yaw - current_yaw
            diff = math.atan2(math.sin(diff), math.cos(diff))
            if abs(diff) < 0.02:
                break
            msg = Twist()
            msg.angular.z = speed if diff > 0 else -speed
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        self.stop_robot()

    def get_yaw(self):
        """从四元数提取航向角"""
        if self.curr_pose is None:
            return 0.0
        q = self.curr_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wait_for_odom(self, timeout_sec=5.0):
        """等待有效里程计数据"""
        self.get_logger().info("等待有效的 /odom 数据...")
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            if self.curr_pose is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
        return False

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.is_running = False
```

---

## 十、参考文献

- Habitat-Sim 代码：
  - `habitat-sim/src_python/habitat_sim/agent/agent.py`
  - `habitat-sim/src_python/habitat_sim/agent/controls/default_controls.py`
  - `habitat-sim/src_python/habitat_sim/agent/controls/object_controls.py`
  - `habitat-sim/src_python/habitat_sim/simulator.py`

- Habitat-Sim 文档：
  - [Habitat-Sim 官方文档](https://aihabitat.org/docs/habitat-sim/)
  - [Navigation](https://aihabitat.org/docs/habitat-sim/habitat_sim.nav.html)

- O3DE 文档：
  - [Recast Navigation](https://docs.o3de.org/docs/user-guide/interactivity/navigation-and-pathfinding/recast-navigation/)

---

**文档版本**：v1.0
**最后更新**：2025-01-29
**作者**：Claude Code
**项目**：nav-eval (Navigation Evaluation Framework)
