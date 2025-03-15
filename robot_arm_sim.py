import pybullet as p
import pybullet_data
import time
import math
import numpy as np

class RobotArmSimulation:
    def __init__(self):
        # Initialize PyBullet
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load robot arm - using a standard KUKA arm model from PyBullet
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        
        # Get number of joints
        self.num_joints = p.getNumJoints(self.robot_id)
        
        # Get joint info
        self.joint_info = {}
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            self.joint_info[i] = {
                'name': info[1].decode('utf-8'),
                'type': info[2],
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            }
        
        # Set the camera position
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=50,
            cameraPitch=-35,
            cameraTargetPosition=[0, 0, 0.5]
        )
    
    def move_to_joint_position(self, target_positions, duration=2.0, steps=100):
        """Move the robot arm to target joint positions."""
        # Get current joint positions
        current_positions = [p.getJointState(self.robot_id, i)[0] for i in range(self.num_joints)]
        
        # Only update the provided target positions, keep others at current position
        full_targets = list(current_positions)
        for joint_idx, position in target_positions.items():
            if 0 <= joint_idx < self.num_joints:
                full_targets[joint_idx] = position
        
        # Interpolate between current and target positions
        for step in range(steps + 1):
            t = step / steps  # Normalized time from 0 to 1
            
            # Perform the interpolation
            interpolated_positions = [
                current_positions[i] + (full_targets[i] - current_positions[i]) * t
                for i in range(self.num_joints)
            ]
            
            # Set joint positions
            for i in range(self.num_joints):
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=interpolated_positions[i],
                    force=self.joint_info[i]['max_force'],
                    maxVelocity=self.joint_info[i]['max_velocity']
                )
            
            # Step the simulation
            p.stepSimulation()
            time.sleep(duration / steps)
    
    def move_in_cartesian_space(self, target_position, target_orientation=None, duration=2.0, steps=100):
        """Move the end effector to the target position and orientation using IK."""
        # Get the ID of the end effector
        end_effector_index = self.num_joints - 1
        
        # If no orientation specified, use current orientation
        if target_orientation is None:
            # Get current position and orientation
            link_state = p.getLinkState(self.robot_id, end_effector_index)
            target_orientation = link_state[1]  # Current orientation
        
        # Calculate joint positions for the target using IK
        joint_positions = p.calculateInverseKinematics(
            self.robot_id, 
            end_effector_index,
            target_position,
            target_orientation
        )
        
        # Create dictionary of joint targets
        target_dict = {i: joint_positions[i] for i in range(len(joint_positions))}
        
        # Move to target joint positions
        self.move_to_joint_position(target_dict, duration, steps)
    
    def add_object(self, position, size=0.05, mass=0.1, color=None):
        """Add a cube as a manipulable object."""
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[size/2, size/2, size/2],
            rgbaColor=color if color else [0.9, 0.1, 0.1, 1]
        )
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[size/2, size/2, size/2]
        )
        object_id = p.createMultiBody(
            baseMass=mass,
            baseInertialFramePosition=[0, 0, 0],
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=position
        )
        return object_id
    
    def run_simulation(self, duration=10):
        """Run the simulation for a specified duration."""
        start_time = time.time()
        while time.time() - start_time < duration:
            p.stepSimulation()
            time.sleep(1/240)  # Simulate at 240Hz
    
    def close(self):
        """Close the PyBullet connection."""
        p.disconnect()
