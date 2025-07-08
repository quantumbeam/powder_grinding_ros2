# CLAUDE.md
必ず日本語で回答してください
This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Environment

This is a ROS2 Humble workspace for a powder grinding robot system. The project uses Docker for containerized development and supports multiple Universal Robot (UR) models.

## Build Environment
python環境の関係で、ament_pythonは使わない(=setup.pyを使わない)でください。
必ず、ament_cmakeでパッケージを構築してください。
本パッケージはvenv環境でのpython環境を想定していますが、ament_pythonを使った場合にシステムのpythonが使われるのでトラブルの元になります。


## Build Commands

### Docker Environment Setup
```bash
# Build Docker image
cd ./env && ./BUILD-DOCKER-IMAGE.sh

# Run Docker container with Terminator (multiple terminals)
cd ./env && ./LAUNCH-TERMINATOR-TERMINAL.sh

# Run Docker container (single terminal)
cd ./env && ./RUN-DOCKER-CONTAINER.sh
```

### ROS2 Workspace Build
```bash
# Initial setup (first time only, run inside Docker container)
./INITIAL_SETUP_ROS_ENVIROMENTS.sh

# Full workspace build
./BUILD_ROS_WORKSPACE.sh

# Quick build (aliased as 'b' in container)
b
```

### Colcon Build Commands
```bash
# Standard build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --cmake-clean-cache

# Parallel build (adjust workers for performance)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8 --cmake-clean-cache
```

## Testing

The project includes standard ROS2 testing:
- `ament_lint_auto` and `ament_lint_common` for linting
- Copyright and flake8 tests in individual packages
- PEP257 documentation style tests

## Package Architecture

### Core Packages
- **grinding_robot_bringup**: Robot startup and launch files
- **grinding_robot_description**: URDF models and robot descriptions
- **grinding_moveit_config**: MoveIt configuration for motion planning
- **grinding_robot_control**: Controller wrappers and control interfaces
- **grinding_motion_routines**: Motion generation and execution logic
- **grinding_force_torque**: Force/torque sensor data processing and filtering
- **grinding_scene_description**: Environment and planning scene management

### Package Structure
Each package follows standard ROS2 conventions:
- `package.xml`: Package metadata and dependencies
- `CMakeLists.txt`: Build configuration
- `launch/`: Launch files for starting nodes
- `config/`: Configuration YAML files
- `src/`: Source code (C++ and Python)

### Key Launch Files
- `grinding_robot_bringup/launch/ur5e_with_pestle_bringup.launch.py`: Main robot startup
- `grinding_robot_description/launch/view_ur5e_with_pestle.launch.py`: Model visualization
- `grinding_moveit_config/launch/ur_moveit.launch.py`: MoveIt planning interface

## Dependencies

### Python Dependencies
- Managed through `requirements.txt` and rosdep
- Use rosdep-registered packages when possible in `package.xml`
- Avoid pip installs in favor of rosdep for ROS2 compatibility

### External Dependencies
- Third-party packages managed via `third_party.repos`
- UR driver and MoveIt2 installed via binary packages
- pytracik library for inverse kinematics

## Development Notes

### Docker Usage
- GPU support available with `nvidia` option
- X11 forwarding enabled for GUI applications
- Persistent container volumes for development

### Common Issues
- Build freezing on low-spec machines: reduce `--parallel-workers`
- Python module path issues: use rosdep over pip when possible
- WSL2 performance: limit parallel build workers

### Robot Support Status
- ✅ UR5e (fully implemented)
- 🔄 UR3e, FR3, Cobotta series (planned)

## Component Integration

### Force/Torque Processing
- WrenchFilter component for sensor data filtering
- WrenchConverter for coordinate frame transformations
- Statistics collection for calibration

### Motion Planning
- MoveIt2 integration for path planning
- Joint trajectory controller interface
- Custom motion routines for grinding operations

### Coordinate Frames
- TF2 transformations for sensor data
- Robot-specific kinematic configurations
- End-effector (pestle) positioning