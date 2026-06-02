import os

file_path = '/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/worlds/tomato_farm_px4_complete.sdf'

try:
    with open(file_path, 'r') as f:
        content = f.read()
        
    if '<start_paused>1</start_paused>' in content:
        content = content.replace('<start_paused>1</start_paused>', '<start_paused>0</start_paused>')
        with open(file_path, 'w') as f:
            f.write(content)
        print("Successfully set the world to AUTO-PLAY! Gazebo will no longer start paused.")
    else:
        print("Could not find <start_paused>1</start_paused>. Maybe already fixed?")
except Exception as e:
    print(f"Error: {e}")
