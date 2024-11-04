import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'visual_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Inclure tous les fichiers de lancement.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='st5dronelab',
    maintainer_email='st5dronelab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vp_node = visual_processing.vp_node:main",
            'republish = visual_processing.republish:main',
        ],
    },
)