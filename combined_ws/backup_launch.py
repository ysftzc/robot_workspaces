import shutil

src = '/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/launch/combined_gazebo.launch.py'
dst = '/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/launch/combined_gazebo_BACKUP.launch.py'

shutil.copy(src, dst)
print(f"Backup created at: {dst}")
