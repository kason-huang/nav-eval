#!/usr/bin/env python3
"""
O3DE Simulator - VLNEvaluation

This module provides an O3DE-based simulator interface for Vision-Language Navigation
evaluation, following the Habitat-style Env interface (reset/step/close).
"""

import math
import time
import json
import importlib
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple, List
from enum import Enum

import numpy as np

# Import Episode from env.py
from .env import Episode

# ROS2 imports (optional, for testing without ROS2)
try:
    import rclpy
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    # Allow module to be imported without ROS2 for testing
    rclpy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    QoSDurabilityPolicy = None
    Image = None
    CvBridge = None
    ROS2_AVAILABLE = False

# O3DE imports (optional)
try:
    # Check if o3de_sim is available by trying to import constants
    spec = importlib.util.find_spec("o3de_sim.constants")
    if spec is not None:
        import o3de_sim.o3de_api.o3de_agent as o3de_agent
        import o3de_sim.o3de_api.interface.math as o3de_math
        from o3de_sim import constants
        # Lazy load scene_builder only when needed
        BasicSceneBuilder = None
        O3DE_AVAILABLE = True
    else:
        raise ImportError()
except:
    # Allow module to be imported without o3de_sim for testing
    o3de_agent = None
    o3de_math = None
    constants = None
    BasicSceneBuilder = None
    O3DE_AVAILABLE = False


# =============================================================================
# Data Structures
# =============================================================================

class ActionType(Enum):
    """Action types for VLN"""
    STOP = 0
    FORWARD = 1
    LEFT = 2
    RIGHT = 3


# =============================================================================
# Utility Functions
# =============================================================================

def dict_to_vector3(d: Dict[str, float]) -> 'o3de_math.Vector3':
    """Convert dict to Vector3"""
    if o3de_math is None:
        return d
    return o3de_math.Vector3(d['x'], d['y'], d['z'])


def euler_to_quaternion(euler_deg: Dict[str, float]) -> 'o3de_math.Quaternion':
    """
    Convert Euler angles (degrees) to Quaternion

    Args:
        euler_deg: {'x': 0, 'y': 0, 'z': 0} in degrees

    Returns:
        Quaternion(w, x, y, z)
    """
    if o3de_math is None:
        return None

    x, y, z = [math.radians(d) for d in [euler_deg['x'], euler_deg['y'], euler_deg['z']]]

    # ZYX Euler sequence (commonly used in robotics)
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


def euclidean_distance(pos1: Dict[str, float], pos2: Dict[str, float]) -> float:
    """Calculate Euclidean distance between two positions"""
    return math.sqrt(
        (pos1['x'] - pos2['x'])**2 +
        (pos1['y'] - pos2['y'])**2 +
        (pos1['z'] - pos2['z'])**2
    )


def vector3_to_dict(vec: 'o3de_math.Vector3') -> Dict[str, float]:
    """Convert Vector3 to dict"""
    if vec is None:
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
    return {'x': vec.x, 'y': vec.y, 'z': vec.z}


# =============================================================================
# Sensor Subscriber (ROS2)
# =============================================================================

class SensorSubscriber:
    """
    ROS2 sensor data subscriber for RGB and Depth observations only.

    Note: LaserScan and collision detection are handled by VLNActionExecutor
    in action_executor.py to avoid duplicate subscriptions.
    """

    def __init__(self, qos_profile=None):
        """
        Initialize sensor subscriber

        Args:
            qos_profile: ROS2 QoS profile for sensor topics
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 is not available. Please install ROS2 or use MockO3DESimulator for testing.")

        if qos_profile is None:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )

        self.node = rclpy.create_node('o3de_sensor_subscriber')
        self.bridge = CvBridge()

        # Data storage
        self.latest_rgb = None
        self.latest_depth = None
        self._rgb_updated = False
        self._depth_updated = False

        # Create subscribers
        self._create_subscribers(qos_profile)

    def _create_subscribers(self, qos_profile: QoSProfile):
        """Create ROS2 topic subscribers"""
        # RGB image subscription
        self.rgb_sub = self.node.create_subscription(
            Image,
            '/rgb',
            self._rgb_callback,
            qos_profile
        )

        # Depth image subscription
        self.depth_sub = self.node.create_subscription(
            Image,
            '/depth',
            self._depth_callback,
            qos_profile
        )

    def _rgb_callback(self, msg: Image):
        """RGB image callback"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._rgb_updated = True
        except Exception as e:
            self.node.get_logger().warning(f'Failed to convert RGB: {e}')

    def _depth_callback(self, msg: Image):
        """Depth image callback"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self._depth_updated = True
        except Exception as e:
            self.node.get_logger().warning(f'Failed to convert Depth: {e}')

    def get_observation(self) -> Dict[str, np.ndarray]:
        """
        Get observation after an action completes.

        This method should be called AFTER marking for new data with
        mark_for_new_data(). It waits for at least one new RGB and Depth
        frame to arrive, then returns the latest data.

        Returns:
            {'rgb': ndarray(H,W,3), 'depth': ndarray(H,W)}
        """
        # Wait for at least one new frame to arrive after action completes
        timeout = 5.0
        start_time = time.time()

        while not (self._rgb_updated and self._depth_updated):
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if time.time() - start_time > timeout:
                raise TimeoutError('Timeout waiting for sensor data after action')

        return {
            'rgb': self.latest_rgb.copy(),
            'depth': self.latest_depth.copy()
        }

    def get_current_observation(self) -> Optional[Dict[str, np.ndarray]]:
        """
        Get current observation immediately without checking update flags.

        Returns the latest RGB and Depth data available, or None if no data
        has been received yet. Use this when you just want the current data
        without waiting for new frames.

        Returns:
            {'rgb': ndarray(H,W,3), 'depth': ndarray(H,W)} or None
        """
        if self.latest_rgb is None or self.latest_depth is None:
            return None

        return {
            'rgb': self.latest_rgb.copy(),
            'depth': self.latest_depth.copy()
        }

    def mark_for_new_data(self):
        """
        Mark that we want new sensor data.

        Call this AFTER executing an action. It resets the update flags so
        that we wait for the next frame(s) to arrive.
        """
        self._rgb_updated = False
        self._depth_updated = False


# =============================================================================
# Action Executor Wrapper
# =============================================================================

class ActionExecutorWrapper:
    """
    Wrapper for action execution using VLNActionExecutor from action_executor.py

    This wraps the tested VLNActionExecutor class to execute discrete actions:
    - 0: STOP
    - 1: FORWARD 25cm
    - 2: LEFT 15°
    - - 3: RIGHT 15°
    """

    def __init__(self, linear_speed: float = 0.1, angular_speed: float = 0.3):
        """
        Initialize action executor

        Args:
            linear_speed: Linear velocity for forward motion (m/s)
            angular_speed: Angular velocity for rotation (rad/s)
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 is not available. Please install ROS2 or use MockO3DESimulator for testing.")

        # Import VLNActionExecutor from action_executor.py
        from . import action_executor
        self.executor = action_executor.VLNActionExecutor()

        # Set speed parameters (executor has default values, we can override them)
        self.executor.LINEAR_SPEED = linear_speed
        self.executor.ANGULAR_SPEED = angular_speed

    def execute_action(self, action: int):
        """
        Execute discrete action

        Args:
            action: Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        if action == ActionType.STOP.value:
            self.stop()
        elif action == ActionType.FORWARD.value:
            self.executor.execute_forward_25cm()
        elif action == ActionType.LEFT.value:
            self.executor.execute_rotate_15deg('left')
        elif action == ActionType.RIGHT.value:
            self.executor.execute_rotate_15deg('right')
        else:
            self.executor.get_logger().warning(f'Unknown action: {action}')

    def execute_forward_25cm(self):
        """Move forward 25cm"""
        self.executor.execute_forward_25cm()

    def execute_rotate_15deg(self, direction: str = 'left'):
        """
        Rotate 15 degrees

        Args:
            direction: 'left' or 'right'
        """
        self.executor.execute_rotate_15deg(direction)

    def _move_timed(self, linear_x: float = 0.0, angular_z: float = 0.0, duration_sec: float = 1.0):
        """Move for a specified duration"""
        self.executor.move_timed(linear_x, angular_z, duration_sec)

    def stop(self):
        """Stop the robot"""
        self.executor.stop_robot()

    def get_collision_count(self) -> int:
        """Get collision count from executor"""
        return self.executor.collision_count


# =============================================================================
# O3DE Agent Manager
# =============================================================================

class O3DEAgentManager:
    """Manages O3DE Agent connection and operations"""

    def __init__(self, mode: str = constants.O3DE_AGENT_SOCKET_MODE if constants else 'socket'):
        """
        Initialize O3DE Agent Manager

        Args:
            mode: O3DE communication mode ('peb' or 'socket')
        """
        if o3de_agent is None:
            raise RuntimeError('o3de_sim is not available. Please ensure o3de_sim is installed.')

        self.mode = mode
        self.agent = o3de_agent.O3DEAgent(mode=mode)
        self.robot_entity_id = None
        self.scene_loaded = False

    def create_scene(self, scene_config_path: str):
        """
        Load scene from config file

        Args:
            scene_config_path: Path to scene_config.json
        """
        with open(scene_config_path, 'r') as f:
            scene_info = json.load(f)

        builder = BasicSceneBuilder(scene_info, self.agent)
        builder.build_scene()
        self.scene_loaded = True

    def create_robot(self, name: str = 'robot', asset_path: str = None,
                    position: Dict[str, float] = None, rotation: Dict[str, float] = None):
        """
        Create robot entity

        Args:
            name: Robot entity name
            asset_path: Path to robot mesh asset (optional)
            position: Initial position
            rotation: Initial rotation (Euler angles in degrees)
        """
        # Create entity at position
        pos = dict_to_vector3(position or {'x': 0, 'y': 0, 'z': 0})
        self.robot_entity_id = self.agent.editor_tool.create_new_entity_at_position(pos, name)

        # Set rotation if provided
        if rotation:
            quat = euler_to_quaternion(rotation)
            self.agent.tc.set_world_rotation(self.robot_entity_id, quat)


    def set_robot_position(self, position: Dict[str, float], rotation: Dict[str, float] = None):
        """
        Set robot position and rotation

        Args:
            position: Position dict
            rotation: Rotation dict (Euler angles in degrees, optional)
        """
        if self.robot_entity_id is None:
            raise RuntimeError('Robot entity not created')

        pos = dict_to_vector3(position)
        self.agent.tc.set_world_position(self.robot_entity_id, pos)

        if rotation:
            quat = euler_to_quaternion(rotation)
            self.agent.tc.set_world_rotation(self.robot_entity_id, quat)


    def get_robot_position(self) -> Optional[Dict[str, float]]:
        """
        Get robot position via O3DE API

        Returns:
            Position dict {'x': ..., 'y': ..., 'z': ...} or None if not available
        """
        if self.robot_entity_id is None:
            return None

        pos = self.agent.tc.get_world_position(self.robot_entity_id)
        return vector3_to_dict(pos)

    def get_robot_entity_id(self):
        """Get robot entity ID"""
        return self.robot_entity_id


# =============================================================================
# O3DE Simulator
# =============================================================================

class O3DESimulator:
    """
    O3DE-based simulator for VLN evaluation

    Habitat-style interface: reset() -> step() -> close()
    """

    def __init__(self,
                 mode: str = 'socket',
                 success_threshold: float = 0.2,
                 collision_threshold: float = 0.3,
                 linear_speed: float = 0.1,
                 angular_speed: float = 0.3):
        """
        Initialize O3DE Simulator

        Args:
            mode: O3DE communication mode ('peb' or 'socket')
            success_threshold: Distance threshold for success (meters)
            collision_threshold: Distance threshold for collision detection (meters)
            linear_speed: Linear velocity (m/s)
            angular_speed: Angular velocity (rad/s)
        """
        # Check availability
        if not O3DE_AVAILABLE:
            raise RuntimeError("O3DE is not available. Please install o3de_sim or use MockO3DESimulator for testing.")
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 is not available. Please install ROS2 or use MockO3DESimulator for testing.")

        # Configuration
        self.mode = mode
        self.success_threshold = success_threshold
        self.collision_threshold = collision_threshold

        # Initialize O3DE Agent Manager
        self.o3de_manager = O3DEAgentManager(mode=mode)

        # Initialize action executor (contains VLNActionExecutor which handles ROS2)
        self.action_executor = ActionExecutorWrapper(
            linear_speed=linear_speed,
            angular_speed=angular_speed
        )

        # Initialize sensor subscriber for RGB and Depth observations
        # Note: VLNActionExecutor already handles LaserScan for collision detection
        self.sensor_sub = SensorSubscriber()

        # Episode state
        self.current_episode: Optional[Episode] = None
        self.current_step: int = 0
        self.goal_position: Optional[Dict[str, float]] = None
        self.trajectory: List[Dict[str, float]] = []

    def reset(self, episode: Episode) -> Dict[str, np.ndarray]:
        """
        Reset simulator with new episode

        Args:
            episode: Episode data

        Returns:
            Initial observation {'rgb': ..., 'depth': ...}
        """
        self.current_episode = episode
        self.current_step = 0
        self.goal_position = episode.goal_position
        self.trajectory = []

        # Load scene if config path provided
        if episode.scene_config_path:
            self.o3de_manager.create_scene(episode.scene_config_path)

        # Create or update robot
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

        # Wait for sensors to be ready
        time.sleep(0.5)

        # Get initial observation (wait for data to be available)
        obs = self.sensor_sub.get_observation()

        # Record initial position
        pos = self.o3de_manager.get_robot_position()
        if pos:
            self.trajectory.append(pos)

        return obs

    def step(self, action: int) -> Tuple[Dict[str, np.ndarray], float, bool, Dict[str, Any]]:
        """
        Execute one step

        Args:
            action: Action ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)

        Returns:
            obs: {'rgb': ..., 'depth': ...}
            reward: 0.0 (not used in VLN)
            done: bool (episode finished)
            info: dict with position, distance_to_goal, collision, step
        """
        # Execute action (synchronous blocking call)
        self.action_executor.execute_action(action)

        # Mark for new data after action completes
        # This ensures we wait for data that arrives AFTER the action
        self.sensor_sub.mark_for_new_data()

        # Get observation (waits for at least one new RGB and Depth frame)
        obs = self.sensor_sub.get_observation()

        # Get robot position via O3DE API
        position = self.o3de_manager.get_robot_position()

        # Calculate distance to goal
        distance_to_goal = euclidean_distance(position, self.goal_position)

        # Get collision count from ActionExecutorWrapper (VLNActionExecutor)
        collision_count = self.action_executor.get_collision_count()
        collision = collision_count > 0

        # Update step count
        self.current_step += 1

        # Record trajectory
        if position:
            self.trajectory.append(position)

        # Check if done
        done = False
        if action == ActionType.STOP.value and distance_to_goal < self.success_threshold:
            done = True

        # Check timeout
        if self.current_episode.max_steps and self.current_episode.max_steps:
            done = True

        # Build info
        info = {
            'position': position,
            'distance_to_goal': distance_to_goal,
            'collision': collision,
            'collision_count': collision_count,
            'step': self.current_step
        }

        return obs, 0.0, done, info

    def close(self):
        """Close simulator and cleanup resources"""
        # Stop robot
        self.action_executor.stop()

        # Destroy ROS2 node
        if hasattr(self, 'sensor_sub') and self.sensor_sub.node:
            self.sensor_sub.node.destroy_node()

        # Shutdown ROS2
        try:
            rclpy.shutdown()
        except:
            pass

    def get_current_position(self) -> Optional[Dict[str, float]]:
        """Get current robot position"""
        return self.o3de_manager.get_robot_position()

    def get_distance_to_goal(self) -> float:
        """Calculate distance to goal"""
        pos = self.get_current_position()
        if pos is None or self.goal_position is None:
            return float('inf')
        return euclidean_distance(pos, self.goal_position)

    def get_trajectory(self) -> List[Dict[str, float]]:
        """Get trajectory"""
        return self.trajectory.copy()


# =============================================================================
# Main
# =============================================================================

if __name__ == '__main__':
    # Simple test
    print('O3DE Simulator Module')
    print('This module provides O3DESimulator for VLN evaluation')
    print('')
    print('Usage:')
    print('  from o3de_simulator import O3DESimulator, Episode')
    print('  sim = O3DESimulator()')
    print('  obs = sim.reset(episode)')
    print('  obs, reward, done, info = sim.step(action)')
    print('  sim.close()')
