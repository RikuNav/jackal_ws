import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'joycon'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes Ricardo Navarro',
    maintainer_email='paolo.alfonso.reyes@gmail.com ricardonavarro2003@gmail.com',
    description='Joycon to velocity message',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy2bot = joycon.joy2bot:main'
        ],
    },
)
