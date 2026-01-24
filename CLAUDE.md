# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a navigation evaluation framework that combines O3DE (Open 3D Engine) simulation with vision-language navigation (VLN) models. The repository consists of:

1. **o3de_sim/** - O3DE simulation interface library for Python
2. Root-level Python modules for VLN evaluation, action execution, policy, and environment abstraction

## Key Architecture

### O3DE Communication Modes

The `o3de_sim` library provides two communication modes for interacting with O3DE:

- **PEB Mode** (`constants.O3DE_AGENT_PEB_MODE`): Process-internal communication using Python Editor Bindings (PEB). Used when running Python scripts directly within O3DE's editor environment.
- **Socket Mode** (`constants.O3DE_AGENT_SOCKET_MODE`): Remote communication via HTTP/WebSocket. Used for controlling O3DE from an external Python process.

Mode selection is handled by `O3DEAgent` in `o3de_sim/o3de_api/o3de_agent.py`:

```python
agent = o3de_agent.O3DEAgent(mode=constants.O3DE_AGENT_SOCKET_MODE)
# or
agent = o3de_agent.O3DEAgent(mode=constants.O3DE_AGENT_PEB_MODE)
```

### O3DE Agent Tools

Regardless of mode, `O3DEAgent` exposes a unified API via these tool classes:
- `entity_tool` - Entity creation, search, manipulation
- `editor_tool` - Level management, entity creation at positions
- `tc` (TransformComponent) - Position, rotation, scale operations
- `pc` (PhysicsComponent) - Rigid body and collider configuration
- `render_tool` - Camera captures and rendering
- `asset_tool` - Asset management
- `prefab_tool` - Prefab operations
- `general` - Utility functions including `wait_for_propagation()`
- `editor_layer_python_tool` - Editor layer Python operations

### O3DE Extras Modules

The extras modules provide specialized functionality:

**ROS2 Integration** (`o3de_api/peb_extras/` or `o3de_api/interface_extras/`):
- `ros2_tool` - ROS2 camera sensor creation and image capture
- `simulation_tool` - Simulation control (enter/exit game mode, physics toggle)

Usage:
```python
from o3de_sim.o3de_api.peb_extras import ros2_tool, simulation_tool
# Or for socket mode:
# from o3de_sim.o3de_api.interface_extras import ros2_tool, simulation_tool

simulation_tool.enter_game_mode()
simulation_tool.toggle_physics(True)
```

### Scene Building

The `BasicSceneBuilder` class (`o3de_sim/common/scene_builder/basic_scene_builder.py`) provides declarative scene construction from JSON configs:

1. Scene configuration is defined in `scene_config.json` format
2. `validator.py` validates scene configs
3. Builder spawns entities, configures transforms, physics, materials based on config

### Root-Level Modules

- **action_executor.py** - ROS2-based action executor for VLN robot control via `/cmd_vel`, `/odom`, `/scan` topics
- **eval.py** - VLN model evaluation framework using Habitat simulation
- **env.py** - Abstract environment interface (`Env` class, `MockEnv` implementation)
- **policy.py** - Abstract policy interface with `WebSocketPolicy` (remote policy server) and `MockPolicy`
- **sim.py** - (currently empty placeholder)

## Common Commands

### Running O3DE Agent Examples

```bash
# PEB mode (run within O3DE editor Python environment)
cd o3de_sim/examples/peb
python api_usage.py
python scene_build.py

# Socket mode (requires O3DE with socket server running)
cd o3de_sim/examples/socket
python api_usage.py
```

### Running VLN Action Executor

```bash
# Requires ROS2 environment with O3DE publishing sensor data
python action_executor.py
# Interactive commands: 1=forward 25cm, 2=left 15°, 3=right 15°, f/b/l/r=timed moves, q=quit
```

### Running VLN Evaluation

```bash
python eval.py \
    --model_path <path_to_model> \
    --habitat_config_path config/vln_r2r.yaml \
    --eval_split val_unseen \
    --output_path ./results/val_unseen/streamvln \
    --num_frames 32 \
    --num_history 8
```

## Important Patterns

### O3DE Synchronization

After making changes to entities or components in O3DE, always call:
```python
agent.general.wait_for_propagation()
```

This ensures O3DE processes changes before subsequent operations.

### Transform Operations

Position/rotation are handled via the `tc` (TransformComponent):
```python
# Set position
agent.tc.set_position(entity_id, o3de_math.Vector3(x, y, z))

# Set rotation (quaternion)
agent.tc.set_rotation(entity_id, o3de_math.Quaternion(w, x, y, z))
```

### Physics Configuration

Physics components use property paths defined in `constants.py`:
```python
# For static rigid bodies
agent.pc.enable_physx_static_rigid_body(entity_id)

# For dynamic rigid bodies
agent.pc.enable_physx_dynamic_rigid_body(entity_id)

# Colliders
agent.pc.set_physx_mesh_collider(entity_id)
```

### Asset Path Resolution

Asset paths are relative to the O3DE project root. Use:
```python
from o3de_sim.config import O3DE_PROJECT_PATH
```

### ROS2 Camera Capture

For capturing RGB images from ROS2 cameras in game mode:
```python
from o3de_sim.o3de_api.peb_extras import ros2_tool
from o3de_sim import constants

# Capture camera output to file
ros2_tool.capture_rgb(
    camera_name="my_camera",
    save_path="/path/to/output.png",
    namespace_strategy=constants.ROS2FrameNamespaceStrategy.Default,
    frame_name="sensor_frame"
)
```

The frame_id is constructed based on namespace_strategy:
- Default (0): `{camera_name}_{frame_name}`
- Empty (1): `{frame_name}`
- FromEntityName (2): `{camera_name}_{frame_name}`
- Custom (3): `{namespace}_{frame_name}` (requires `namespace` parameter)

### Action Space

VLN actions are integer-coded: `0=STOP`, `1=FORWARD`, `2=LEFT`, `3=RIGHT` (see `eval.py` `actions2idx`).

### QoS Configuration

When subscribing to O3DE ROS2 topics, use SENSOR_DATA QoS for compatibility:
```python
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)
```

## Scene Config Structure

Scene configs (see `o3de_sim/common/scene_builder/scene_config_extras.json` for full documentation) define:

Required fields:
- `scene_name` - Level name (string)
- `scene_type` - Scene type like "desktop" (string)
- `objects` - Array of objects to spawn

Object properties:
- `name` - Entity name (string, required)
- `translate` - World position `{x, y, z}` (required)
- `rotate` - XYZ Euler rotation in degrees `{x, y, z}` (required)
- `scale` - Scale factors `{x, y, z}` (optional)
- `asset_path` - Relative path to mesh GLTF file (required)
- `material_asset_path` - Material to apply to Default Material (optional)
- `model_materials` - Array of material paths for Model Materials slots (optional, overrides `material_asset_path`)
- `rigid_body_type` - Physics type: `"static"` or `"dynamic"` (optional, defaults to no physics)

Lights (optional array):
- `name` - Light entity name
- `translate` - Position `{x, y, z}`
- `rotate` - Rotation `{x, y, z}`
- `light_type` - Enum: 1=Point(Sphere), 2=Spot(Disk), 3=Capsule, 4=Quad, 5=Polygon, 6=Point(SimplePunctual), 7=Spot(SimplePunctual)
- `intensity_mode` - Enum: 0=Lumen, 1=Candela, 2=Ev100, 3=Nit (default 0)
- `intensity` - Light intensity float (default 100)
- `color` - RGB color `{r, g, b}` (default white)

## Others
Habitat-lab contains some well-developed designs for navigation competitions that you can refer to. The directory is `habitat-lab`; this directory is for reference only and should not be directly used in your project.