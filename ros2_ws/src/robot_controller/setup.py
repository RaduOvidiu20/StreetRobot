from setuptools import setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include fișierele de lansare
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),  # Include fișierele URDF/RViz
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),  # Adaugă marker
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ovidiu',
    maintainer_email='ovidiu@example.com',
    description='A description of the robot_controller package',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'motor_controller = robot_controller.motor_controller:main',
            'serial_reader = robot_controller.serial_reader:main',
            'static_transform_publisher = robot_controller.static_transform_publisher:main',
            'pointcloud_to_laserscan = robot_controller.pointcloud_to_laserscan:main',
        ],
    },
)
