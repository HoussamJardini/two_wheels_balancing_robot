from setuptools import setup

package_name = 'two_wheeled_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_robot.py']),
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for two-wheeled self-balancing robot with obstacle avoidance',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = two_wheeled_robot.balance_controller:main',
            'keyboard_control = two_wheeled_robot.keyboard_control:main',
            'obstacle_avoidance = two_wheeled_robot.obstacle_avoidance:main',
        ],
    },
)
