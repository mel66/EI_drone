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
            'MoveLeft = behavior.Command_float_setter:MoveLeftmain',
            'MoveForward = behavior.Command_float_setter:MoveForwardmain',
            'MoveBackward = behavior.Command_float_setter:MoveBackwardmain',
            'MoveRight = behavior.Command_float_setter:MoveRightmain',
            'TurnLeft = behavior.Command_float_setter:TurnLeftmain',
            'TurnRight = behavior.Command_float_setter:TurnRightmain',
            'MoveUp = behavior.Command_float_setter:MoveUpmain',
            'MoveDown = behavior.Command_float_setter:MoveDownmain',
            'AlignCorridor = behavior.Semi_aut:AlignCorridormain',
            'CenterCorridor = behavior.Semi_aut:CenterCorridormain',
            'MoveForwardVp = behavior.Semi_aut:MoveForwardVpmain',
            'UTurn = behavior.Semi_aut:UTurnmain',
            'SlideRight = behavior.Semi_aut:SlideRightmain',
            'SlideLeft = behavior.Semi_aut:SlideLeftmain',            
            'DoorCrossingLeft = behavior.Semi_aut:DoorCrossingLeftmain',
            'DoorCrossingRight = behavior.Semi_aut:DoorCrossingRightmain',

        ]
    },
)
