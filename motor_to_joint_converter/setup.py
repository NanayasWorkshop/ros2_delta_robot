from setuptools import find_packages, setup

package_name = 'motor_to_joint_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_to_joint_converter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuuki',
    maintainer_email='yuuzena@gmail.com',
    description='Converts motor positions to joint angles for delta robot visualization',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_to_joint_converter_node = motor_to_joint_converter.motor_to_joint_converter_node:main',
        ],
    },
)
