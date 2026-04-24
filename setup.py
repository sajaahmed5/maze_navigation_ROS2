from setuptools import setup
import os
from glob import glob

package_name = 'maze_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@university.edu',
    description='Maze Navigation using Potential Field Method',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'potential_field_planner = maze_navigation.potential_field_planner:main',
        ],
    },
)
