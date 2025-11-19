from setuptools import find_packages, setup

package_name = 'checkpoint_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # prefer declaring ROS deps in package.xml; keep only build-time/python packaging deps here
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = checkpoint_pkg.lidar:main',
            'rotate_robot = checkpoint_pkg.rotate_robot:main',
            'move_robot = checkpoint_pkg.move_robot:main',
            'find_opening = checkpoint_pkg.find_opening:main',
            'aruco_pose_node = checkpoint_pkg.aruco_pose_parameters:main',
        ],
    },
)