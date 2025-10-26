from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'interactive_tf_markers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuuki',
    maintainer_email='yuuzena@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interactive_marker_tf_node = interactive_tf_markers.interactive_marker_tf_node:main',
            'tf_listener_example = interactive_tf_markers.tf_listener_example:main',
        ],
    },
)
