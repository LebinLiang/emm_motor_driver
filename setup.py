from setuptools import setup
import os
from glob import glob

package_name = 'emm_motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 driver for EMM V5 Serial Motors',
    license='TODO',
    entry_points={
        'console_scripts': [
            'motor_node = emm_motor_driver.motor_node:main',
            'sync_example = emm_motor_driver.sync_example:main'
        ],
    },
)
