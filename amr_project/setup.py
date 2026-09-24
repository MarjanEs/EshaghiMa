from setuptools import setup
from glob import glob
import os

package_name = 'amr_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[
        package_name,
        package_name + '.planning',
        package_name + '.utils',
        package_name + '.localization'
    ],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@h-brs.de',
    description='AMR Project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'planner_node=amr_project.planning.planner_node:main',
            'localization_node=amr_project.localization.localization_node:main',
            'exploration_node = amr_project.exploration.exploration_node:main',
        ]
    },
)
