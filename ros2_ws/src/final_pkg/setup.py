from setuptools import find_packages, setup

package_name = 'final_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'networkx'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = final_pkg.aruco_detector:main',
            'aruco_pose_node = final_pkg.aruco_pose_parameters:main',
            'lidar = final_pkg.process_lidar:main',
            'localizer = final_pkg.localizer:main',
            'move_robot_follow_path = final_pkg.move_robot_follow_path:main',
            'move_robot_outside_box = final_pkg.move_robot_outside_box:main',
            'move_robot_to_aruco = final_pkg.move_robot_to_aruco:main',
            'move_robot = final_pkg.move_robot:main',
            'navigator = final_pkg.navigator:main',
            'process_lidar = final_pkg.process_lidar:main',
            'robot_manager = final_pkg.robot_manager:main',
            'rotate_robot_to_aruco = final_pkg.rotate_robot_to_aruco:main',
            'rotate_robot = final_pkg.rotate_robot:main',
            'user_interface = final_pkg.user_interface:main',  
        ],
    },
)
