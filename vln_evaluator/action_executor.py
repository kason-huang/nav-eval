#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

# 可选依赖：Contact Sensor 支持
try:
    from gazebo_msgs.msg import ContactsState
    CONTACT_SENSOR_AVAILABLE = True
except ImportError:
    CONTACT_SENSOR_AVAILABLE = False
    ContactsState = None


class VLNActionExecutor(Node):
    def __init__(self, collision_method: str = "auto"):
        """
        Args:
            collision_method: 碰撞检测方式
                - "contact": 优先使用 Contact Sensor (推荐)
                - "scan": 使用 LaserScan
                - "auto": 自动检测 (优先 Contact Sensor，降级到 LaserScan)
                - "both": 同时使用两种方式 (任一触发即停止)
        """
        super().__init__('vln_action_executor')

        # === 碰撞检测方式配置 ===
        self.collision_method = collision_method.lower()
        self._init_collision_detection()

        # === QoS: 与 O3DE 的 SENSOR_DATA 兼容 ===
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, sensor_qos)

        # === 传感器订阅器 ===
        self._init_sensor_subscriptions(sensor_qos)

        # 状态变量
        self.curr_pose = None
        self.collision_detected = False
        self.collision_count = 0
        self.is_running = False
        self.last_collision_source = None  # 记录碰撞来源：'contact' 或 'scan'

        # 参数
        self.LINEAR_SPEED = 0.1
        self.ANGULAR_SPEED = 0.3
        self.COLLISION_THRESHOLD = 0.2

        self._log_collision_config()

    def _init_collision_detection(self):
        """初始化碰撞检测配置"""
        # 检测 Contact Sensor 可用性
        self.use_contact_sensor = CONTACT_SENSOR_AVAILABLE
        self.use_laser_scan = True

        if self.collision_method == "contact":
            if not CONTACT_SENSOR_AVAILABLE:
                self.get_logger().warn(
                    "Contact Sensor 被请求但不可用 (gazebo_msgs 未安装)，降级到 LaserScan"
                )
                self.use_contact_sensor = False
            self.use_laser_scan = False
        elif self.collision_method == "scan":
            self.use_contact_sensor = False
        elif self.collision_method == "both":
            if not CONTACT_SENSOR_AVAILABLE:
                self.get_logger().warn("Contact Sensor 不可用，仅使用 LaserScan")
                self.use_contact_sensor = False
        elif self.collision_method == "auto":
            # 自动检测：优先 Contact Sensor
            if not CONTACT_SENSOR_AVAILABLE:
                self.get_logger().info("Contact Sensor 不可用，使用 LaserScan")
                self.use_contact_sensor = False
        else:
            self.get_logger().warn(
                f"未知的 collision_method: {self.collision_method}，使用 auto 模式"
            )
            self.collision_method = "auto"
            self._init_collision_detection()

    def _init_sensor_subscriptions(self, sensor_qos):
        """初始化传感器订阅器"""
        # Contact Sensor 订阅
        if self.use_contact_sensor:
            self.contact_sub = self.create_subscription(
                ContactsState,
                '/contact_sensor',
                self.contact_callback,
                sensor_qos
            )
            self.get_logger().info("✓ 已订阅 Contact Sensor (/contact_sensor)")

        # LaserScan 订阅
        if self.use_laser_scan:
            self.scan_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                sensor_qos
            )
            self.get_logger().info("✓ 已订阅 LaserScan (/scan)")

    def _log_collision_config(self):
        """输出碰撞检测配置"""
        methods = []
        if self.use_contact_sensor:
            methods.append("Contact Sensor")
        if self.use_laser_scan:
            methods.append("LaserScan")

        mode_str = f"模式: {self.collision_method.upper()}"
        methods_str = " + ".join(methods) if methods else "无"

        print(f"\n{'='*60}")
        print(f"🔧 碰撞检测配置")
        print(f"{'='*60}")
        print(f"  {mode_str}")
        print(f"  使用方式: {methods_str}")
        print(f"{'='*60}\n")

        self.get_logger().info(f"碰撞检测模式: {self.collision_method}, 方式: {methods_str}")

    def odom_callback(self, msg):
        """
        只接受"有效"的 odom 消息：
        - 时间戳 sec != 0（排除初始化零帧）
        - 或位置明显非零（双重保险）
        """
        header = msg.header
        pose = msg.pose.pose
        pos = pose.position

        # 判断是否为有效时间戳（O3DE 物理启动后 stamp.sec > 0）
        if header.stamp.sec == 0:
            # 若时间戳无效，再检查是否是全零位姿（初始默认值）
            quat = pose.orientation
            is_zero_pose = (
                abs(pos.x) < 1e-6 and abs(pos.y) < 1e-6 and abs(pos.z) < 1e-6 and
                abs(quat.x) < 1e-6 and abs(quat.y) < 1e-6 and
                abs(quat.z) < 1e-6 and abs(quat.w - 1.0) < 1e-3
            )
            if is_zero_pose:
                return  # 忽略初始零帧

        # 接受有效数据
        self.curr_pose = pose

    def contact_callback(self, msg: ContactsState):
        """
        Contact Sensor 碰撞回调

        核心逻辑：只要 states 数组非空，就表示发生了物理碰撞
        """
        # 只在任务运行中且尚未标记碰撞时才触发
        if self.is_running and not self.collision_detected:
            # 判断是否有碰撞（states 非空）
            if len(msg.states) > 0:
                self.collision_detected = True
                self.collision_count += 1
                self.last_collision_source = "contact"

                # 提取第一个碰撞的详细信息
                state = msg.states[0]
                collision_info = (
                    f"{state.collision1_name.split('Name:')[-1].strip()} ⟷ "
                    f"{state.collision2_name.split('Name:')[-1].strip()}"
                )

                print(f"！！！检测到碰撞 (Contact Sensor)！！！")
                print(f"   碰撞对象: {collision_info}")
                print(f"   接触点数: {len(state.contact_positions)}")
                print(f"   总次数: {self.collision_count}")
                print(f"{'='*60}")

    def scan_callback(self, msg):
        """LaserScan 碰撞回调"""
        ranges = msg.ranges
        if not ranges:
            return

        num_points = len(ranges)

        # 设定检测点数。为了安全，window 不应超过总点数的一半
        # 否则 ranges[:window] 和 ranges[-window:] 会在后方重叠
        requested_window = 30
        window = min(requested_window, num_points // 2)

        # Python 的切片操作即使 window > len(ranges) 也不会报错（会取到头）
        # 但显式限制 window 可以保证逻辑语义准确
        front_view = list(ranges[:window]) + list(ranges[-window:])

        valid_ranges = [
            r for r in front_view
            if msg.range_min < r < msg.range_max
        ]

        if valid_ranges:
            current_min = min(valid_ranges)
            if current_min < self.COLLISION_THRESHOLD:
                # 只有在任务运行中且尚未标记碰撞时才触发
                if self.is_running and not self.collision_detected:
                    self.collision_detected = True
                    self.collision_count += 1
                    self.last_collision_source = "scan"
                    print(f"！！！检测到碰撞 (LaserScan)！！！")
                    print(f"   最小距离: {current_min:.3f}m")
                    print(f"   总次数: {self.collision_count}")
                    print(f"{'='*60}")

    def get_yaw(self):
        if self.curr_pose is None:
            return 0.0
        q = self.curr_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wait_for_odom(self, timeout_sec=5.0):
        """主动等待有效的 odom 数据"""
        self.get_logger().info("等待有效的 /odom 数据...")
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            if self.curr_pose is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
        return False

    def execute_forward_25cm(self):
        if not self.wait_for_odom():
            self.get_logger().error("超时：未收到有效的 /odom 数据，无法执行精确前进。")
            return
            
        self.get_logger().info("执行动作：前进 25cm（基于 odom）")
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
            msg.linear.x = self.LINEAR_SPEED
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.stop_robot()

    def execute_rotate_15deg(self, direction="left"):
        if not self.wait_for_odom():
            self.get_logger().error("超时：未收到有效的 /odom 数据，无法执行精确旋转。")
            return
            
        angle_rad = math.radians(15) if direction == "left" else -math.radians(15)
        self.get_logger().info(f"执行动作：{direction}转 15度（基于 odom）")
        
        start_yaw = self.get_yaw()
        target_yaw = start_yaw + angle_rad
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
            msg.angular.z = self.ANGULAR_SPEED if diff > 0 else -self.ANGULAR_SPEED
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        self.stop_robot()

    def move_timed(self, linear_x=0.0, angular_z=0.0, duration_sec=1.0):
        self.get_logger().info(f"执行定时移动: linear.x={linear_x}, angular.z={angular_z}, 持续 {duration_sec}s")
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

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        self.is_running = False


def main():
    import argparse

    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='VLN 动作执行器 - 支持 Contact Sensor 和 LaserScan 碰撞检测'
    )
    parser.add_argument(
        '--collision-method',
        type=str,
        default='auto',
        choices=['auto', 'contact', 'scan', 'both'],
        help='碰撞检测方式: auto(自动检测), contact(Contact Sensor优先), scan(LaserScan), both(同时使用)'
    )
    args = parser.parse_args()

    rclpy.init()
    node = VLNActionExecutor(collision_method=args.collision_method)

    print("\n" + "="*50)
    print("🚀 VLN 动作执行器（O3DE 仿真专用 - 已修复 odom 问题）")
    print("="*50)
    print("指令说明:")
    print("  1 : 前进 25cm (需 /odom)")
    print("  2 : 左转 15°   (需 /odom)")
    print("  3 : 右转 15°   (需 /odom)")
    print("  f : 前进 1秒    (无需 /odom)")
    print("  b : 后退 1秒    (无需 /odom)")
    print("  l : 左转 1秒    (无需 /odom)")
    print("  r : 右转 1秒    (无需 /odom)")
    print("  q : 退出程序")
    print("-"*50)

    try:
        while rclpy.ok():
            user_input = input("请输入指令: ").strip().lower()
            if user_input == '1':
                node.execute_forward_25cm()
            elif user_input == '2':
                node.execute_rotate_15deg("left")
            elif user_input == '3':
                node.execute_rotate_15deg("right")
            elif user_input == 'f':
                node.move_timed(linear_x=0.2, duration_sec=1.0)
            elif user_input == 'b':
                node.move_timed(linear_x=-0.2, duration_sec=1.0)
            elif user_input == 'l':
                node.move_timed(angular_z=0.6, duration_sec=1.0)
            elif user_input == 'r':
                node.move_timed(angular_z=-0.6, duration_sec=1.0)
            elif user_input == 'q':
                break
            else:
                print("⚠️ 无效指令，请重新输入")
    except KeyboardInterrupt:
        print("\n🛑 收到中断信号，正在退出...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
