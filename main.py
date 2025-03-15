import numpy as np
import time
import pybullet as p
import math
import argparse
from robot_arm_sim import RobotArmSimulation

def generate_line_points(start_point, end_point, num_points=20):
    """
    Generate a sequence of points forming a straight line from start to end
    
    Args:
        start_point: [x, y, z] coordinates of the line start
        end_point: [x, y, z] coordinates of the line end
        num_points: Number of points to generate along the line
        
    Returns:
        List of points [x, y, z] along the straight line
    """
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start_point[0] + t * (end_point[0] - start_point[0])
        y = start_point[1] + t * (end_point[1] - start_point[1])
        z = start_point[2] + t * (end_point[2] - start_point[2])
        points.append([x, y, z])
    return points

def generate_circle_points(center, radius, num_points=30):
    """
    Generate a sequence of points forming a circle on the floor
    
    Args:
        center: [x, y, z] coordinates of the circle center
        radius: Radius of the circle
        num_points: Number of points to generate along the circle
        
    Returns:
        List of points [x, y, z] along the circle
    """
    points = []
    for i in range(num_points):
        angle = 2.0 * math.pi * i / num_points
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = center[2]  # Keep the same height as the center
        points.append([x, y, z])
    
    # Add the first point again to close the circle
    points.append(points[0].copy())
    return points

def draw_path_line(prev_point, current_point, line_color=[0, 0, 1], line_width=3.0, lifetime=0):
    """
    Draw a persistent line segment in the simulation
    
    Args:
        prev_point: Starting point of the line segment [x, y, z]
        current_point: Ending point of the line segment [x, y, z]
        line_color: RGB color of the line (default: blue)
        line_width: Width of the line in pixels
        lifetime: How long the line should remain visible (0 = forever)
    """
    # Adjust the z-coordinate to be slightly above the floor to avoid z-fighting
    drawing_height = 0.002
    p1 = [prev_point[0], prev_point[1], drawing_height]
    p2 = [current_point[0], current_point[1], drawing_height]
    
    # Draw a line using PyBullet's debug visualization
    p.addUserDebugLine(
        p1, 
        p2, 
        lineColorRGB=line_color,
        lineWidth=line_width,
        lifeTime=lifetime
    )

def get_perpendicular_orientation():
    """
    Calculate an orientation quaternion that makes the end-effector perpendicular to the ground
    
    Returns:
        A quaternion [x, y, z, w] representing an orientation where the z-axis of 
        the end-effector points directly downward (perpendicular to the ground)
    """
    # The quaternion for pointing the z-axis of the end-effector downward
    # This corresponds to the tool pointing directly at the ground
    # For KUKA iiwa, pointing downward is a rotation of 180 degrees around the y-axis
    return p.getQuaternionFromEuler([0, math.pi, 0])

def draw_line_with_robot(sim, line_points, pen_down_height=0.01, pen_up_height=0.1):
    """
    Draw a line by moving the robot arm through a sequence of points
    
    Args:
        sim: RobotArmSimulation instance
        line_points: List of [x, y, z] points defining the line
        pen_down_height: Height for drawing (pen down)
        pen_up_height: Height for moving without drawing (pen up)
    """
    # Create a pen attachment for the robot end effector
    pen_id = sim.add_object(position=[0, 0, 0], size=0.02, mass=0.01, color=[0, 0, 1, 1])
    
    # Get the end effector index
    end_effector_index = sim.num_joints - 1
    
    # Get orientation that keeps the end-effector perpendicular to the ground
    perpendicular_orientation = get_perpendicular_orientation()
    
    # Move to starting position but stay above the drawing plane
    start_approach = line_points[0].copy()
    start_approach[2] = pen_up_height  # Raise Z to approach height
    sim.move_in_cartesian_space(start_approach, target_orientation=perpendicular_orientation, duration=1.0)
    
    # Connect the pen to the end effector
    pen_constraint = p.createConstraint(
        parentBodyUniqueId=sim.robot_id,
        parentLinkIndex=end_effector_index,
        childBodyUniqueId=pen_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0.1],
        childFramePosition=[0, 0, 0]
    )
    
    # Draw each segment of the line
    prev_point = None
    
    for i, point in enumerate(line_points):
        # Adjust Z-height for drawing on the floor
        drawing_point = point.copy()
        drawing_point[2] = pen_down_height
        
        # Move to the point, always keeping the end-effector perpendicular to the ground
        sim.move_in_cartesian_space(drawing_point, target_orientation=perpendicular_orientation, duration=0.3)
        
        # Draw path line for all points except the first
        if prev_point is not None:
            draw_path_line(prev_point, drawing_point)
        
        prev_point = drawing_point.copy()
        
        # Small pause for visualization
        time.sleep(0.05)
    
    # Lift the arm after drawing, maintaining perpendicular orientation
    end_point = line_points[-1].copy()
    end_point[2] = pen_up_height
    sim.move_in_cartesian_space(end_point, target_orientation=perpendicular_orientation, duration=1.0)

def draw_straight_line_demo(sim):
    """
    Demo function for drawing a straight line
    
    Args:
        sim: RobotArmSimulation instance
    """
    # Define line to draw (start and end points)
    start_point = [0.5, 0.3, 0.01]  # x, y, z
    end_point = [0.5, -0.3, 0.01]    # x, y, z
    
    print("Drawing a straight line using inverse kinematics...")
    print(f"Start point: {start_point}")
    print(f"End point: {end_point}")
    
    # Generate points for the line
    num_points = 30
    line_points = generate_line_points(start_point, end_point, num_points)
    
    # Place visual markers at the start and end points
    start_marker = sim.add_object(position=start_point, size=0.03, mass=0, color=[1, 0, 0, 1])  # Red
    end_marker = sim.add_object(position=end_point, size=0.03, mass=0, color=[0, 1, 0, 1])     # Green
    
    # Draw a preview of the line path on the floor
    for i in range(len(line_points)-1):
        preview_line = draw_path_line(
            line_points[i], 
            line_points[i+1], 
            line_color=[0.5, 0.5, 0.5],  # Gray preview line
            line_width=1.0
        )
    
    # Draw the line with the robot arm
    draw_line_with_robot(sim, line_points)

def draw_circle_demo(sim):
    """
    Demo function for drawing a circle
    
    Args:
        sim: RobotArmSimulation instance
    """
    # Define circle parameters - placed further from the robot base with smaller radius
    center = [0.6, 0.0, 0.01]  # x, y, z center of the circle (increased x coordinate)
    radius = 0.15             # Radius of the circle in meters (reduced from 0.25)
    
    print("Drawing a circle using inverse kinematics...")
    print(f"Center point: {center}")
    print(f"Radius: {radius} meters")
    
    # Generate points for the circle
    num_points = 40
    circle_points = generate_circle_points(center, radius, num_points)
    
    # Place visual marker at the center
    center_marker = sim.add_object(position=center, size=0.03, mass=0, color=[1, 0, 0, 1])  # Red
    
    # Draw a preview of the circle path on the floor
    for i in range(len(circle_points)-1):
        preview_line = draw_path_line(
            circle_points[i], 
            circle_points[i+1], 
            line_color=[0.5, 0.5, 0.5],  # Gray preview line
            line_width=1.0
        )
    
    # Draw the circle with the robot arm
    draw_line_with_robot(sim, circle_points)

def main():
    """
    Main function to run the robot arm drawing demo
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Robot Arm Drawing Demo')
    parser.add_argument('--demo', type=str, default='line', 
                        choices=['line', 'circle'],
                        help='Type of demo to run (line or circle)')
    args = parser.parse_args()
    
    # Initialize the robot arm simulation
    sim = RobotArmSimulation()
    
    # Wait for physics to stabilize
    for _ in range(100):
        p.stepSimulation()
    
    # First position the robot in a convenient initial pose with end-effector perpendicular to ground
    initial_joint_positions = {
        0: 0,    # Base joint rotation
        1: 0.5,  # Shoulder joint
        2: 0,
        3: -1.0, # Elbow joint
        4: 0,
        5: 0.5,  # Wrist joint
        6: 0     # End effector rotation
    }
    sim.move_to_joint_position(initial_joint_positions, duration=1.0)
    
    # Run the selected demo
    if args.demo == 'line':
        draw_straight_line_demo(sim)
    elif args.demo == 'circle':
        draw_circle_demo(sim)
    
    # Run the simulation for a while to see the results
    print("Drawing completed. Press Ctrl+C to exit.")
    try:
        sim.run_simulation(duration=10)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        sim.close()
        print("Simulation ended.")

if __name__ == "__main__":
    main()
