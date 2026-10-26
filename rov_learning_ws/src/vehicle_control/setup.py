from setuptools import find_packages, setup

package_name = 'vehicle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tms',
    maintainer_email='tms@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_command_monitor = vehicle_control.vehicle_command_monitor:main',
            'test_command_publisher = vehicle_control.test_command_publisher:main',
            'ground_station_node = vehicle_control.ground_station_node:main',
        ],
    },
)
