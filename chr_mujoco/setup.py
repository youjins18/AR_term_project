"""Packaging metadata for the CHR MuJoCo ROS node."""

from setuptools import find_packages, setup

package_name = 'chr_mujoco'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/simulator.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'mujoco'],
    zip_safe=True,
    maintainer='youjins18',
    maintainer_email='chldbwls08@seoultech.ac.kr',
    description='MuJoCo plant and ROS 2 interface for CHR.',
    entry_points={'console_scripts': ['simulator_node=chr_mujoco.simulator_node:main']},
)
