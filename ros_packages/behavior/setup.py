from setuptools import find_packages, setup

package_name = 'behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/behavior_launch.xml']),  # Change the filename as needed
    ],
    install_requires=['setuptools','rclpy'],
    zip_safe=True,
    maintainer='edmond',
    maintainer_email='edmond.jean@student-cs.Fr',
    description='Behavior package for drone behaviors',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'fake_hover = behavior.fake_hover:main',
            'fake_forward = behavior.fake_forward:main',
            'status_viewer = behavior.status_viewer:main',
            'joy_teleop = behavior.joy_teleop:main',
            'command = behavior.command:main',
            'TakeOff = behavior.TakeOff:main',
            'Land = behavior.Land:main',
            'Hover = behavior.Hover:main',
            'MoveLeft = behavior.MoveLeft:main',
            'MoveForward = behavior.MoveForward:main',
            'MoveBackward = behavior.MoveBackward:main',
            'MoveRight = behavior.MoveRight:main',
            'TurnLeft = behavior.TurnLeft:main',
            'TurnRight = behavior.TurnRight:main',
            'MoveUp = behavior.MoveUp:main',
            'MoveDown = behavior.MoveDown:main',

        ]
    },
)
