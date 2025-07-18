from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'blocksword_planner'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/' + package_name, ['package.xml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    # Tutti i launch file nella cartella launch/
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Blocksworld HTN planner using PyHOP for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'htn_planner_node = blocksword_planner.blocksword_node:main'
        ],
    },
)
