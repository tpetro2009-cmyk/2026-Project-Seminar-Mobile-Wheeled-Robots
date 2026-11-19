
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'webots_amcl_localization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.wbt'),
        ),
        (
            os.path.join('share', package_name, 'protos'),
            glob('protos/*.proto'),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*'),
        ),
        (
            os.path.join('share', package_name, 'webots'),
            glob('webots/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tpetro2009-cmyk',
    maintainer_email='tpetro2009@gmail.com',
    description='AMCL localization experiments for TurtleBot3 in Webots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_recorder = webots_amcl_localization.experiment_recorder:main',
            'trajectory_publisher = webots_amcl_localization.trajectory_publisher:main',
        ],
    },
)
