from setuptools import find_packages, setup

package_name = 'controllers'

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
    maintainer='israel',
    maintainer_email='israel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ttc_node = controllers.ttc_node:main',
            'pid_node = controllers.pid_node:main',
            'gap_node = controllers.gap_node:main',
            'purepursuit_node = controllers.purepursuit_node:main',
            'rrt_node = controllers.rrt_node:main',
            'mpc_node = controllers.mpc_node:main',
            'square_controller = controllers.square_controller:main',
            'lidar_d = controllers.lidar_d:main',
            'lidar_analysis = controllers.lidar_analysis_node:main',
            'emergency_brake_node = controllers.emergency_brake_node:main',
            'velocity_listener = controllers.velocity_listener:main',
            'velocity_and_odom = controllers.velocity_and_odom:main',
            'control_proyecto = controllers.control_proyecto:main',

        ],
    },
)
