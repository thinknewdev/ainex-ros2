import os
from glob import glob

from setuptools import setup

package_name = 'ainex_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thinknewdev',
    maintainer_email='flashfreakmx@gmail.com',
    description='ROS2 Humble camera bringup for the Hiwonder AiNex head camera.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
