from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'diffusion_policy_inference'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('*.sh')),
        ('lib/' + package_name, glob('scripts/model_inference_node')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yukun Wang',
    maintainer_email='lambert.wang.23@ucl.ac.uk',
    description='ROS 2 package for running diffusion policy inference in simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'model_inference_node = diffusion_policy_inference.model_inference_node:main',
        ],
    },
    py_modules=[
        'diffusion_policy_inference.model_inference_node',
    ],
)
