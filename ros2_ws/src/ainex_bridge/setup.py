from setuptools import setup

package_name = 'ainex_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/bridge.launch.py', 'launch/sim_adapter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AiNex ROS2 Port Maintainers',
    maintainer_email='ainex-ros2@users.noreply.github.com',
    description='ROS2 facade bridging the AiNex vendor surface to motiond',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'bridge = ainex_bridge.bridge_node:main',
            'sim_adapter = ainex_bridge.sim_adapter:main',
        ],
    },
)
