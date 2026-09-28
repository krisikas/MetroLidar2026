import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'metro_web_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'web'), glob('web/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MetroLidar2026 Team',
    maintainer_email='engineer@metrolidar2026.moscow',
    description='Standalone Web Dashboard & 3D FPV Viewer for MetroLidar2026',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_gui_node = metro_web_gui.web_gui_node:main',
        ],
    },
)
