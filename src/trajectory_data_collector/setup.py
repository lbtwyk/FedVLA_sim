from setuptools import find_packages, setup

package_name = 'trajectory_data_collector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wangyukun',
    maintainer_email='lambert.wang.23@ucl.ac.uk',
    description='ROS 2 package for collecting trajectory data for diffusion policy training',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_logger_node = trajectory_data_collector.state_logger_node:main',
        ],
    },
)
