from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'low_level_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pierres',
    maintainer_email='pierre.sainctavit@student-cs.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "twist2pic = low_level_control.twist2pic:main",
            "joy_teleop = low_level.control.joy_teleop:main",
        ],
    },
)
