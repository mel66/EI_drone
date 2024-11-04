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
           'fake_hover = behavior.base_behavior:FakeHoverBehavior',
            'fake_forward = behavior.base_behavior:FakeForwardBehavior',
        ]
    },
)
