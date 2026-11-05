import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'odometry_study'


def tree(dest, src):
    """Install a whole directory tree (protos with meshes/textures/icons)."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            rel = os.path.relpath(root, src)
            target = os.path.join('share', package_name, dest) if rel == '.' \
                else os.path.join('share', package_name, dest, rel)
            out.append((target, [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/resource',
         glob('resource/*.urdf') + glob('resource/*.yaml')),
    ] + tree('protos', 'protos'),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'cmd_publisher = odometry_study.cmd_publisher:main',
        'experiment_recorder = odometry_study.experiment_recorder:main',
    ]},
)
