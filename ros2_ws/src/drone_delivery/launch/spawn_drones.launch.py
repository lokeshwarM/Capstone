import os
from launch import LaunchDescription
# pyrefly: ignore [missing-import]
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    # Path to the drone SDF model
    sdf_path = '/mnt/d/Capstone/drone.sdf'
    
    # Coordinates for the 4 drones in the corners of the 2x2 grid.
    # Grid is 2x2, spacing is 15. So centers are around +/- 7.5
    drones = [
        {'name': 'drone1', 'x': '-7.5', 'y': '-7.5', 'z': '0.5'},
        {'name': 'drone2', 'x': '7.5', 'y': '-7.5', 'z': '0.5'},
        {'name': 'drone3', 'x': '-7.5', 'y': '7.5', 'z': '0.5'},
        {'name': 'drone4', 'x': '7.5', 'y': '7.5', 'z': '0.5'},
    ]
    
    for drone in drones:
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', drone['name'],
                '-file', sdf_path,
                '-x', drone['x'],
                '-y', drone['y'],
                '-z', drone['z'],
                '-robot_namespace', drone['name']
            ],
            output='screen'
        )
        ld.add_action(spawn_node)
        
    return ld
