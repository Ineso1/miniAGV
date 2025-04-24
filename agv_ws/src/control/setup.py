from setuptools import setup
import os
from glob import glob


package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = control.controller:main',
            'odometry = control.odometry:main',
            'trajectory = control.trajectory:main',      
            'circle_trajectory = control.circle_trajectory:main'            
        ],
    },
    rosidl_modules=['msg']
)
