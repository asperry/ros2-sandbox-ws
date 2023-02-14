import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'urdf_tutorial_r2d2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob('launch/*.py')),
        (os.path.join("share", package_name), glob('urdf/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Sperry',
    maintainer_email='adam.sperry@outlook.com',
    description='ROS 2 URDF tutorial package for simulating a walking robot with robot_state_publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = urdf_tutorial_r2d2.state_publisher:main'
        ],
    },
)
