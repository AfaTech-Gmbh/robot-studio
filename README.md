# 3D Robot Arm Simulation with PyBullet

This project demonstrates a 3D robot arm simulation using PyBullet. It provides a Python interface to control a virtual KUKA robot arm, allowing for joint and Cartesian space movement.

## Requirements

- Python 3.8 or higher
- PyBullet 3.2.5
- NumPy 1.24.3

## Installation

1. Clone the repository:

```bash
git clone https://github.com/AfaTech-Gmbh/robot-studio.git
cd robot-studio
```

2. Install the required packages:

```bash
pip install -r requirements.txt
```

3. Run the demo script:

```bash
python main.py
```

## Features

- **3D Visualization**: Real-time rendering of the robot arm and environment
- **Joint Control**: Direct manipulation of each robot joint
- **Cartesian Control**: End-effector positioning using inverse kinematics
- **Object Manipulation**: Pick and place capabilities with physics simulation
- **Motion Planning**: Smooth trajectory generation between points
- **Collision Detection**: Automatic detection and response to collisions

## Usage

### Basic Example

```python
from robot_arm_simulation import RobotArmSimulation

# Initialize the simulation
sim = RobotArmSimulation()

# Move robot joints to specific angles (radians)
sim.move_to_joint_position([0, 0.5, 0, -0.5, 0, 0], duration=2.0, steps=50)

# Move end-effector to a Cartesian position
position = [0.5, 0, 0.5]  # x, y, z in meters
orientation = [0, 0, 0, 1]  # quaternion
sim.move_in_cartesian_space(position, orientation, duration=2.0, steps=50)

# Add an object to manipulate
object_id = sim.add_object([0.5, 0, 0], size=[0.05, 0.05, 0.05], mass=0.1, color=[1, 0, 0, 1])

# Run the simulation for 10 seconds
sim.run_simulation(10.0)

# Cleanup
sim.close()
```

### Advanced Functionality

- **Path Planning**: Create complex movement sequences with waypoints
- **Object Grasping**: Control the gripper to pick up and release objects
- **State Observation**: Get joint states, end-effector position, and object states
- **Programmatic Control**: Integrate with custom control algorithms

## Project Structure

```
robot-studio/
├── main.py                  # Main script for running the simulation
├── robot_arm_simulation.py  # Main simulation class
├── requirements.txt         # Project dependencies
└── .gitignore               # Git ignore file
```

## Extending the Project

To extend this project, you could:
- Add custom URDF robot models
- Implement advanced path planning algorithms
- Create specialized grasping mechanisms
- Add simulated cameras or force sensors
- Implement reinforcement learning for autonomous operation

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- PyBullet team for their excellent physics simulation library
- KUKA for the robot arm model specifications
