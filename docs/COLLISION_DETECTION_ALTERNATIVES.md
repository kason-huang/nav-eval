# VLN碰撞检测方案：不依赖LaserScan的替代方案

## 一、概述

当前 `vln_evaluator/action_executor.py` 使用 2D 激光雷达（`/scan` 话题）进行碰撞检测，检测范围为前方 0.2m。

本文档提供不依赖 LaserScan 的碰撞检测替代方案，适用于以下场景：
- O3DE 场景未配置激光雷达传感器
- 需要更高精度的碰撞检测
- 希望减少传感器依赖

---

## 二、方案对比总览

| 方案 | 实现难度 | 实时响应 | 精度 | 静态/动态 | ROS2依赖 | 推荐度 |
|------|---------|---------|------|----------|---------|--------|
| **Contact Sensor** | 低 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 均支持 | ✅ | ⭐⭐⭐⭐⭐ |
| **Depth Camera** | 中 | ⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 均支持 | ✅ | ⭐⭐⭐⭐ |
| **Raycasting** | 高 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 仅静态 | ❌ | ⭐⭐⭐ |
| **Odometry Drift** | 低 | ⭐⭐ | ⭐⭐ | 均支持 | ✅ | ⭐⭐ |
| **Recast Navigation** | 高 | ⭐ | ⭐⭐⭐ | 仅静态 | ❌ | ⭐（仅辅助）|

---

## 三、Recast Navigation 方案分析（不推荐单独使用）

### 3.1 概述

**Recast Navigation Gem** 是 O3DE 提供的导航网格和路径查找系统，主要用于AI寻路和路径规划。

### 3.2 核心功能

| 功能 | 描述 | API示例 |
|------|------|---------|
| **创建导航网格** | 从物理碰撞体生成可行走区域 | `UpdateNavigationMesh()` |
| **路径查找** | 在两个位置之间找到可行走的路径 | `FindPathBetweenEntities()` |
| **路径查询** | 检查某个位置是否在导航网格内 | `FindPathBetweenPositions()` |

### 3.3 为什么不适合 VL N 碰撞检测

#### ❌ 关键限制

| 限制 | 说明 | 对 VL N 的影响 |
|--------|------|---------------|
| **非实时检测** | 导航网格是预计算的静态网格 | 无法检测动态障碍物 |
| **无碰撞事件** | 没有回调机制通知碰撞发生 | 无法立即响应碰撞 |
| **路径查询，非距离查询** | 只能找到路径，无法返回前方距离 | 无法实现"前方0.2m内停止" |
| **静态场景假设** | 导航网格假设场景不变 | 不适合 VL N 的探索场景 |
| **更新开销大** | 更新导航网格是重量级操作 | 无法实时更新 |

#### ✅ 可以实现的功能

虽然不适合实时碰撞检测，但可以用于**辅助功能**：

1. **路径有效性验证**
   - 在执行动作前检查目标位置是否在导航网格内
   - 避免机器人移动到不可达区域

2. **避障路径规划**
   - 使用 Detour Navigation 找到绕过障碍物的路径
   - 适用于需要预知路径的场景

### 3.4 O3DE 配置步骤（如需使用）

**注意**：这需要通过 O3DE PEB API 调用，无法直接在 `action_executor.py` 中使用。

```python
# 在 O3DE Editor 中配置
1. 创建实体，添加 Recast Navigation Mesh 组件
2. 添加必需组件：Recast Navigation PhysX Provider
3. 添加 Axis Aligned Box Shape 定义导航区域
4. 调用 UpdateNavigationMesh() 生成导航网格
5. 为机器人添加 Detour Navigation 组件用于路径查询
```

**Python API 调用示例**（需要在 O3DE PEB 环境中）：
```python
from Gems.RecastNavigation.Code import RecastNavigation.DetourNavigationBus

# 查找路径
waypoints = DetourNavigationBus.EventResult(
    waypoints,
    detour_entity_id,
    &DetourNavigationBus.Events.FindPathBetweenPositions,
    start_position,
    goal_position
)
```

### 3.5 混合使用建议

**如果同时需要 Recast Navigation 和实时碰撞检测**：

```python
class VLNActionExecutor(Node):
    def __init__(self):
        # 实时碰撞检测（主要）
        self.contact_sub = self.create_subscription(Contacts, '/contact', ...)
        
        # 导航网格查询（辅助）- 需要通过 O3DE API 调用
        self.detour_component = None  # 初始化时获取

    def execute_forward_25cm(self):
        # 1. 计算目标位置
        target_pos = self.calculate_target_position()
        
        # 2. 检查目标是否在导航网格内（预判）
        if not self.is_walkable(target_pos):
            print("目标位置不可行走，跳过动作")
            return
        
        # 3. 执行动作（依赖碰撞传感器检测实际碰撞）
        self.is_running = True
        while not self.collision_detected:
            # ... 控制逻辑 ...
```

### 3.6 推荐度

| 场景 | 推荐度 | 理由 |
|------|---------|------|
| **单独用于碰撞检测** | ⭐ | 不适合，响应延迟高 |
| **辅助路径规划** | ⭐⭐⭐ | 可以验证目标可达性 |
| **混合方案：NavMesh + Contact Sensor** | ⭐⭐⭐⭐ | 结合优势：路径预判 + 实时碰撞 |

---

## 四、方案1：O3DE Contact Sensor（推荐）

### 3.1 概述

**O3DE ROS2 Sensors Gem** 原生支持 **Contact Sensor**，这是最直接、最精确的碰撞检测方案。

### 3.2 优势

- ✅ **原生支持**：O3DE 直接提供，无需额外开发
- ✅ **精确检测**：物理引擎级别的碰撞，100%准确
- ✅ **ROS2集成**：直接发布 `/contact` 话题，无需转换
- ✅ **性能最优**：不依赖传感器数据流，零延迟
- ✅ **配置简单**：在 O3DE Editor 中添加组件即可

### 3.3 O3DE 配置步骤

1. **在 O3DE Editor 中打开场景**
2. **为机器人实体添加 ROS2 Contact Sensor 组件**：
   - 选中机器人 entity
   - Add Component → ROS2 → ROS2 Contact Sensor
3. **配置 Collision Layer**（可选）：
   - 在 PhysX Configuration 中设置碰撞层
   - 确保机器人与障碍物在同一碰撞层
4. **进入 Game Mode**：
   - 确保物理引擎启动
   - Contact Sensor 只在仿真运行时工作

### 3.4 ROS2 话题格式

```python
# 话题名称
/contact

# 消息类型
# 需要确认 O3DE ROS2 Gem 使用的具体消息类型
# 可能是 gazebo_msgs/Contacts 或自定义类型
```

**注意**：Contact Sensor 可能需要安装 `gazebo_msgs` 包：
```bash
sudo apt install ros-${ROS_DISTRO}-gazebo-msgs
```

### 3.5 实现代码

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
# 根据实际 O3DE 发布的消息类型调整
# from gazebo_msgs.msg import Contacts

class VLNActionExecutor(Node):
    def __init__(self):
        super().__init__('vln_action_executor')

        # === QoS 配置 ===
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        # === 替换：订阅 Contact Sensor ===
        # 需要根据 O3DE 实际发布的消息类型调整
        self.contact_sub = self.create_subscription(
            Contacts, '/contact', self.contact_callback, sensor_qos
        )

        # 状态变量
        self.curr_pose = None
        self.collision_detected = False
        self.collision_count = 0
        self.is_running = False

    def contact_callback(self, msg):
        """
        Contact Sensor 碰撞回调
        """
        if not self.is_running or self.collision_detected:
            return

        # 检测到接触即视为碰撞
        # 根据实际消息结构调整判断逻辑
        if msg.states:  # 或其他字段名
            self.collision_detected = True
            self.collision_count += 1
            print(f"检测到碰撞！总次数: {self.collision_count}")

        # 其他动作方法（execute_forward_25cm, execute_rotate_15deg）保持不变
        # 只需移除 scan_callback 即可
```

### 3.6 验证方法

```bash
# 检查话题是否发布
ros2 topic list | grep contact

# 查看话题类型
ros2 topic info /contact

# 监听碰撞数据
ros2 topic echo /contact
```

---

## 四、方案2：Depth Camera（RGB-D）

### 4.1 概述

O3DE Camera Sensor 支持 **Depth Channel**，通过深度图像检测前方障碍物距离。

### 4.2 优势

- ✅ **视觉方式**：利用已有 RGB-D 相机，无需额外传感器
- ✅ **丰富信息**：深度图提供 3D 空间信息，可检测多方向障碍物
- ✅ **研究常用**：VLN 领域广泛使用深度信息进行避障
- ✅ **灵活可调**：可自定义检测区域和阈值

### 4.3 O3DE 配置步骤

1. **为机器人添加 ROS2 Camera Sensor**：
   - 选中机器人 entity
   - Add Component → ROS2 → ROS2 Camera Sensor
   - 确保启用 **Depth** 通道（默认启用）
2. **设置相机参数**：
   - FOV（视场角）：建议 60-90 度
   - 分辨率：推荐 640x480 或 320x240
   - 更新频率：10-30 Hz
3. **安装 cv_bridge**（用于图像转换）：
   ```bash
   sudo apt install ros-${ROS_DISTRO}-cv-bridge
   ```

### 4.4 ROS2 话题格式

```python
# 话题名称
/depth

# 消息类型
sensor_msgs/Image

# 编码格式
16UC1 (16-bit unsigned depth values)
```

### 4.5 实现代码

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        # === 替换：订阅深度相机 ===
        self.depth_sub = self.create_subscription(
            Image, '/depth', self.depth_callback, sensor_qos
        )

        self.bridge = CvBridge()

        # 状态变量
        self.curr_pose = None
        self.collision_detected = False
        self.collision_count = 0
        self.is_running = False

        # 参数
        self.LINEAR_SPEED = 0.1
        self.ANGULAR_SPEED = 0.3
        self.COLLISION_THRESHOLD = 0.5  # 米

    def depth_callback(self, msg):
        """
        深度相机碰撞检测
        """
        if not self.is_running or self.collision_detected:
            return

        try:
            # 转换 ROS2 Image 到 OpenCV 格式
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

            # 转换深度值为米（O3DE 可能使用毫米或其他单位）
            # 需要根据实际数据调整转换公式
            depth_meters = depth_image.astype(np.float32) / 1000.0  # 假设单位是毫米

            # === 检测策略1：关注图像中心区域（机器人前方） ===
            h, w = depth_meters.shape
            roi = depth_meters[h//3:2*h//3, w//3:2*w//3]  # 中心 1/3 区域

            # 过滤无效深度值（0 或 inf）
            valid_depths = roi[roi > 0]

            if len(valid_depths) > 0:
                min_dist = np.min(valid_depths)

                if min_dist < self.COLLISION_THRESHOLD:
                    self.collision_detected = True
                    self.collision_count += 1
                    print(f"深度检测到碰撞！距离: {min_dist:.2f}m")

            # === 检测策略2（可选）：关注前方扇形区域 ===
            # 适用于已知机器人朝向的场景
            # center_x = w // 2
            # forward_roi = depth_meters[h//2:2*h//3, center_x-50:center_x+50]
            # valid_depths_forward = forward_roi[forward_roi > 0]
            # if len(valid_depths_forward) > 0:
            #     min_dist_forward = np.min(valid_depths_forward)
            #     if min_dist_forward < self.COLLISION_THRESHOLD:
            #         self.collision_detected = True

        except Exception as e:
            self.get_logger().warn(f"深度图像处理错误: {e}")

    def odom_callback(self, msg):
        """里程计回调（保持不变）"""
        # ... 原有 odom_callback 逻辑 ...

    # 其他方法（execute_forward_25cm, execute_rotate_15deg）保持不变
```

### 4.6 参数调优

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `COLLISION_THRESHOLD` | 0.5m | 碰撞检测阈值，建议 0.3-0.8m |
| ROI区域 | 中心1/3 | 可根据机器人宽度调整 |
| 深度转换系数 | /1000.0 | 需要根据 O3DE 实际单位调整 |

### 4.7 验证方法

```bash
# 查看深度图像
ros2 topic hz /depth

# 转换并显示（测试用）
ros2 run image_tools image_view image:=/depth
```

---

## 五、方案3：Physics Raycasting（高级方案）

### 5.1 概述

通过 O3DE Physics Engine 直接进行射线检测，不依赖任何传感器。

### 5.2 优势

- ✅ **零传感器依赖**：完全基于物理引擎
- ✅ **精确检测**：射线碰撞检测精度极高
- ✅ **灵活配置**：可自定义检测方向、距离、射线数量
- ✅ **低开销**：不涉及图像处理，CPU占用低

### 5.3 缺点

- ❌ **实现复杂**：需要使用 O3DE PEB API，在 Python 脚本中编写
- ❌ **无 ROS2 接口**：需要在 O3DE 内部运行，无法外部订阅
- ❌ **调试困难**：物理引擎调用难以调试

### 5.4 实现方式

**注意**：此方案需要在 O3DE Editor 中运行 Python 脚本，无法直接在 `action_executor.py` 中实现。

```python
# O3DE PEB 脚本（在 O3DE Editor Python Console 中运行）
from azlmbr import physics as azPhysics
from azlmbr import bus as azBus
from azlmbr.math import Vector3, Quaternion
import o3de_sim.o3de_api.peb as o3de_api

def check_collision(entity_id, distance_threshold=0.2):
    """
    检测机器人前方是否存在碰撞

    Args:
        entity_id: 机器人 entity ID
        distance_threshold: 检测距离（米）

    Returns:
        bool: 是否检测到碰撞
    """
    # 获取机器人位置和朝向
    position = azPhysics.RigidBodyRequestBus(
        azBus.Event, "GetWorldPosition", entity_id
    )

    rotation = azPhysics.RigidBodyRequestBus(
        azBus.Event, "GetWorldRotation", entity_id
    )

    # 计算前方方向向量（假设机器人沿 +X 轴前进）
    # 根据实际机器人朝向调整
    forward = Vector3(1, 0, 0)
    ray_start = position
    ray_end = position + forward * distance_threshold

    # 执行射线检测
    hit = azPhysics.WorldRequestBus.Event(
        "RayCast", ray_start, ray_end
    )

    # 检查是否击中物体
    if hit and hit.distance < distance_threshold:
        return True

    # 可选：多射线检测（前方的左、中、右三方向）
    # 更好的覆盖检测范围

    return False

# 使用示例
robot_id = o3de_api.entity_tool.get_entity_id_by_name("robot")
if check_collision(robot_id, distance_threshold=0.2):
    print("检测到碰撞！")
```

### 5.5 集成到 VLN 系统

**方案1：O3DE Python 脚本定期发布碰撞状态**
```python
# 在 O3DE 中定期调用 raycast 并发布 ROS2 话题
import rclpy
from geometry_msgs.msg import Bool

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        self.collision_pub = self.create_publisher(Bool, '/collision', 10)

    def publish_collision_status(self):
        msg = Bool()
        msg.data = check_collision(robot_id)
        self.collision_pub.publish(msg)

# action_executor.py 订阅 /collision 话题
self.collision_sub = self.create_subscription(
    Bool, '/collision', self.collision_callback, 10
)
```

**方案2：通过 O3DE API 直接查询（在 O3DESimulator 中实现）**
```python
# 在 o3de_simulator.py 的 step() 中
def step(self, action: int):
    # 执行动作
    self.action_executor.execute_action(action)

    # 直接调用 O3DE API 检测碰撞
    collision = self.check_collision_raycast(self.robot_entity_id)

    # 返回结果
    return obs, reward, done, {'collision': collision, ...}
```

---

## 六、方案4：Odometry Drift Detection（简单方案）

### 6.1 概述

通过比较期望速度与实际速度的差异，间接检测碰撞。

### 6.2 优势

- ✅ **零额外依赖**：只需已有的 `/odom` 数据
- ✅ **实现简单**：逻辑清晰，易于调试
- ✅ **性能优异**：CPU 占用极低

### 6.3 缺点

- ❌ **间接检测**：依赖速度差异，非精确碰撞检测
- ❌ **误报率高**：物理引擎波动、摩擦力等都会影响
- ❌ **检测延迟**：碰撞后需等待速度变化才能检测到

### 6.4 实现代码

```python
class VLNActionExecutor(Node):
    def __init__(self):
        super().__init__('vln_action_executor')

        # ... 其他初始化 ...

        # 额外状态变量
        self.expected_velocity = 0.0
        self.velocity_history = []
        self.DRIFT_THRESHOLD = 0.05  # 速度差异阈值（m/s）
        self.DRIFT_COUNT_THRESHOLD = 3  # 连续多少次低于阈值触发碰撞

    def odom_callback(self, msg):
        """里程计回调（增强版）"""
        # ... 原有里程计逻辑 ...

        # 检测速度漂移
        actual_velocity = msg.twist.twist.linear.x

        if self.expected_velocity > 0.05:  # 只在期望前进时检测
            if actual_velocity < self.DRIFT_THRESHOLD:
                self.velocity_history.append(True)
            else:
                self.velocity_history.append(False)

            # 保持最近 5 次检测记录
            self.velocity_history = self.velocity_history[-5:]

            # 连续多次低于阈值 → 可能碰撞
            if sum(self.velocity_history) >= self.DRIFT_COUNT_THRESHOLD:
                if self.is_running and not self.collision_detected:
                    self.collision_detected = True
                    self.collision_count += 1
                    print(f"速度漂移检测到碰撞！期望: {self.expected_velocity:.2f}m/s, 实际: {actual_velocity:.2f}m/s")
                    self.velocity_history = []  # 重置历史

    def execute_forward_25cm(self):
        """前进动作（修改版）"""
        if not self.wait_for_odom():
            return

        self.get_logger().info("执行动作：前进 25cm（基于 odom）")
        start_x = self.curr_pose.position.x
        start_y = self.curr_pose.position.y
        self.is_running = True
        self.collision_detected = False

        # 设置期望速度
        self.expected_velocity = self.LINEAR_SPEED
        self.velocity_history = []

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            dist = math.sqrt(
                (self.curr_pose.position.x - start_x) ** 2 +
                (self.curr_pose.position.y - start_y) ** 2
            )
            if self.collision_detected or dist >= 0.25:
                break
            msg = Twist()
            msg.linear.x = self.LINEAR_SPEED
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

        # 清理
        self.expected_velocity = 0.0
        self.stop_robot()

    def execute_rotate_15deg(self, direction="left"):
        """旋转动作（速度漂移检测不适用）"""
        # 旋转动作通常不会因碰撞而停止，
        # 可以使用激光雷达或深度相机检测
        # ... 原有代码 ...
```

### 6.5 参数调优

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DRIFT_THRESHOLD` | 0.05 m/s | 速度差异阈值，越小越敏感 |
| `DRIFT_COUNT_THRESHOLD` | 3 | 连续次数阈值，越大越保守 |

---

## 七、混合方案推荐

根据实际场景，可以组合多种方案：

### 7.1 主备方案：Contact Sensor + Depth Camera

**实现方式**：
1. Contact Sensor 作为主要检测（精确碰撞）
2. Depth Camera 作为辅助检测（预碰撞警告）

**代码**：
```python
def contact_callback(self, msg):
    """主要检测：精确碰撞"""
    if msg.states:
        self.collision_detected = True
        self.collision_count += 1

def depth_callback(self, msg):
    """辅助检测：距离警告"""
    min_dist = get_min_distance_from_depth(msg)
    if min_dist < 0.3 and min_dist > self.COLLISION_THRESHOLD:
        self.get_logger().warn(f"接近障碍物！距离: {min_dist:.2f}m")
        # 可选择提前减速或停止
```

### 7.2 分级检测：Raycasting + Contact Sensor

**实现方式**：
1. Raycasting 检测 0.5m 范围（预判）
2. Contact Sensor 检测实际碰撞

---

## 八、选择建议

### 8.1 根据场景选择

| 场景 | 推荐方案 | 理由 |
|------|---------|------|
| **O3DE 仿真环境** | Contact Sensor | 原生支持，最精确 |
| **真实机器人** | Depth Camera | RGB-D 相机常见，兼容性好 |
| **无传感器** | Odometry Drift | 最简单，但精度低 |
| **精确物理仿真** | Raycasting | 物理引擎级别，最可靠 |
| **研究实验** | Depth Camera | 可视化、可调试 |

### 8.2 实现优先级

1. **首选**：Contact Sensor（如果 O3DE 支持）
2. **次选**：Depth Camera（通用性好）
3. **备用**：Odometry Drift（简单但不可靠）
4. **高级**：Raycasting（需要深度定制）

---

## 九、验证流程

无论选择哪种方案，建议按以下流程验证：

### 9.1 功能验证

```bash
# 1. 检查话题发布
ros2 topic list | grep -E "(contact|depth)"

# 2. 查看话题类型和数据
ros2 topic info /contact
ros2 topic echo /contact --once

# 3. 运行 action_executor
python vln_evaluator/action_executor.py
```

### 9.2 碰撞测试

1. **在 O3DE 中放置障碍物**
2. **执行前进动作**
3. **验证**：
   - 碰撞时机器人是否立即停止？
   - `collision_count` 是否增加？
   - 碰撞日志是否正确输出？

### 9.3 阈值调优

1. 逐步调整 `COLLISION_THRESHOLD` 或 `DRIFT_THRESHOLD`
2. 记录碰撞检测距离和实际碰撞距离
3. 选择合适的阈值平衡误报和漏报

---

## 十、参考文献

### O3DE 官方文档
- [ROS 2 Sensors Gem](https://development--o3deorg.netlify.app/docs/user-guide/gems/reference/robotics/ros2sensors)
- [ROS 2 Gem](https://docs.o3de.org/docs/user-guide/gems/reference/robotics/ros2)
- [Python Editor Bindings](https://docs.o3de.org/docs/user-guide/gems/reference/script/python/editor-python-bindings/)

### VLN 碰撞检测研究
- CARE: Collision Avoidance via Repulsive Estimation (2025)
- Safe-VLN: Collision Avoidance for VLN (2024)
- VLN-CM: Zero-Shot VLN with Collision Mitigation (2024)

### RGB-D 避障方法
- RNBF: Real-Time RGB-D Based Neural Barrier Functions (2025)
- RGB-D Obstacle Detection for AGVs (2025)
- Dynamic Obstacle Tracking with RGB-D (2024)

---

## 十一、总结

| 方案 | 推荐度 | 适用场景 | 实现工作量 | 主要用途 |
|------|---------|---------|-----------|----------|
| Contact Sensor | ⭐⭐⭐⭐⭐ | O3DE 仿真 | 低 | 实时碰撞检测 |
| Depth Camera | ⭐⭐⭐⭐ | 通用场景 | 中 | 视觉避障 |
| Raycasting | ⭐⭐⭐ | 精确仿真需求 | 高 | 物理级检测 |
| Odometry Drift | ⭐⭐ | 临时方案 | 低 | 速度异常检测 |
| Recast Navigation | ⭐（仅辅助）| 路径规划需求 | 高 | 路径有效性检查 |

**推荐策略**：
1. **O3DE 仿真**：使用 Contact Sensor（方案1）
2. **无 Contact Sensor**：使用 Depth Camera（方案2）
3. **零传感器需求**：使用 Odometry Drift（方案4）作为临时方案
4. **深度定制**：使用 Raycasting（方案3）实现精确检测
5. **路径规划需求**：使用 Recast Navigation（方案3）作为**辅助**，验证目标可达性

**重要提示**：
- Recast Navigation **不适合单独用于 VL N 碰撞检测**
- 建议作为 Contact Sensor/Depth Camera 的**辅助工具**，用于路径有效性验证
- 混合方案：Contact Sensor（主） + Recast Navigation（辅）可实现最佳效果

---

**文档版本**：v1.0
**最后更新**：2025-01-29
**作者**：Claude Code
**项目**：nav-eval (Navigation Evaluation Framework)
