from glob import glob

from setuptools import find_packages, setup

package_name = 'arm6_ros2'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Arm project maintainers',
    maintainer_email='maintainer@example.com',
    description='ROS 2 hardware control of a six-axis DYNAMIXEL arm and independent gripper.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'arm6_node = arm6_ros2.node:main',
        'forward_line_test = arm6_ros2.forward_line_test:main',
        'encoder_calibration = arm6_ros2.encoder_calibration:main',
        'desktop_control = arm6_ros2.desktop_control:main',
        'gamepad_monitor = arm6_ros2.gamepad_monitor:main',
        'gamepad_control = arm6_ros2.gamepad_control:main',
    ]},
)

