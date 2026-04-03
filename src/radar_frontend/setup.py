from glob import glob
from setuptools import find_packages, setup


package_name = 'radar_frontend'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='metro',
    maintainer_email='metro@local.com',
    description='ROS 2 radar frontend pipeline for RF2O bridging, observation adaptation, and monitored private send.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'observation_adapter_node = radar_frontend.observation_adapter_node:main',
            'private_observation_velocity_sender_node = radar_frontend.private_observation_velocity_sender_node:main',
            'radar_trial_monitor_node = radar_frontend.radar_trial_monitor_node:main',
            'rf2o_radar_bridge_node = radar_frontend.rf2o_radar_bridge_node:main',
        ],
    },
)
