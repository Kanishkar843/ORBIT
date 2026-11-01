from setuptools import setup
import os
from glob import glob

package_name = 'path_planning_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_name + '.nodes'],  # explicit, no find_packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Developer',
    maintainer_email='robot@example.com',
    description='Real-hardware A* navigation stack for the ZLAC8015D diffbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigator = path_planning_pkg.nodes.navigator:main',
            'goal_publisher = path_planning_pkg.nodes.goal_publisher:main',
        ],
    },
)