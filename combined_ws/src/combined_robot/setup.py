from setuptools import setup
import os
from glob import glob

package_name = 'combined_robot'

def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        files = [os.path.join(path, filename) for filename in filenames]
        if files:
            paths.append((os.path.join('share', package_name, path), files))
    return paths

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.xacro')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'rviz'),   glob('rviz/*')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    (os.path.join('share', package_name, 'maps'),   glob('maps/*')),
]
data_files.extend(package_files('models'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mission_manager = combined_robot.mission_manager:main',
            'initial_pose_publisher = combined_robot.initial_pose_publisher:main',
            'pick_place_detachable = combined_robot.pick_place_detachable:main',
            'greenhouse_nearest_pick_place = combined_robot.greenhouse_nearest_pick_place:main',
            'tomato_depth_detector = combined_robot.tomato_depth_detector:main',
            'tomato_depth_mapper = combined_robot.tomato_depth_mapper:main',
            'tomato_map_panel = combined_robot.tomato_map_panel:main',
            'tomato_map_pick_target = combined_robot.tomato_map_pick_target:main',
            'tomato_collision_scene_manager = combined_robot.tomato_collision_scene_manager:main',
            'yolo_tomato_detector = combined_robot.yolo_tomato_detector:main',
            'yolo_bbox_viewer = combined_robot.yolo_bbox_viewer:main',
            'gazebo_tomato_detector = combined_robot.gazebo_tomato_detector:main',
            'greenhouse_planning_scene = combined_robot.greenhouse_planning_scene:main',
            'arm_cartesian_gui = combined_robot.arm_cartesian_gui:main',
            'record_fr3_observation_pose = combined_robot.fr3_observation_pose_recorder:main',
        ],
    },
)
