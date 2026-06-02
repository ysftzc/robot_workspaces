import os

src_file = '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_sync.yaml'
dst_file = '/home/yusuf/robot_workspaces/combined_ws/custom_slam_params.yaml'

try:
    with open(src_file, 'r') as f:
        content = f.read()
    
    # Replace the scan_topic
    content = content.replace('scan_topic: /scan', 'scan_topic: /lidar/scan')
    # Replace base_frame if it defaults to base_footprint
    content = content.replace('base_frame: base_footprint', 'base_frame: base_link')
    
    with open(dst_file, 'w') as f:
        f.write(content)
    print("Successfully created custom_slam_params.yaml!")
except Exception as e:
    print(f"Error: {e}")
