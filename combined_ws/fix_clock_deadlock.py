import os

file_path = '/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/launch/combined_gazebo.launch.py'

try:
    with open(file_path, 'r') as f:
        content = f.read()
        
    # The clock_bridge node should NOT have use_sim_time: True because it causes a deadlock
    old_clock = """    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )"""
    
    new_clock = """    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        # DO NOT USE use_sim_time: True here, or it deadlocks waiting for the clock it is trying to publish!
        output='screen'
    )"""
    
    if old_clock in content:
        content = content.replace(old_clock, new_clock)
        with open(file_path, 'w') as f:
            f.write(content)
        print("Successfully fixed the clock deadlock bug in combined_gazebo.launch.py!")
    else:
        print("Could not find the exact clock_bridge string to replace. Maybe already fixed?")
except Exception as e:
    print(f"Error: {e}")
