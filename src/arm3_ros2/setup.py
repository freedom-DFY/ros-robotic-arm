from glob import glob

from setuptools import find_packages, setup

package_name = 'arm3_ros2'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Arm project maintainers',
    maintainer_email='maintainer@example.com',
    description='Control the first three DYNAMIXEL arm joints with ROS 2.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'arm3_node = arm3_ros2.node:main',
        'cartesian_step = arm3_ros2.cartesian_step:main',
    ]},
)
