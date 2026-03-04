from collections import defaultdict
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'px4_ego_py'

launch_files_by_dir = defaultdict(list)
for launch_file in glob('launch/**/*.launch.py', recursive=True):
    install_dir = os.path.join('share', package_name, os.path.dirname(launch_file))
    launch_files_by_dir[install_dir].append(launch_file)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + list(launch_files_by_dir.items()),
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='hdn',
    maintainer_email='dongnanhu6556@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "offboard_control_test = px4_ego_py.offboard_control_test:main",
            "ds5_mode_teleop = px4_ego_py.ds5_mode_teleop:main",
        ],
    },
)
