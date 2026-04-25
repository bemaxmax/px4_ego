import os

from setuptools import find_packages, setup


package_name = 'detect'


def collect_data_files(source_dir):
    data_files = []

    for root, _, files in os.walk(source_dir):
        if not files:
            continue

        install_dir = os.path.join('share', package_name, root)
        file_paths = [os.path.join(root, file_name) for file_name in files]
        data_files.append((install_dir, file_paths))

    return data_files


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for resource_dir in ('models',):
    data_files.extend(collect_data_files(resource_dir))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'numpy', 'requests'],
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
            'target_detector = detect.target_detector_node:main',
        ],
    },
)
