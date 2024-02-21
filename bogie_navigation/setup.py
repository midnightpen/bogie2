from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'bogie_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'config'),glob('config/*.lua')),
        (os.path.join('share',package_name,'config'),glob('config/*.yaml')),
        (os.path.join('share',package_name,'config'),glob('config/*.pgm')),
        (os.path.join('share',package_name,'maps'),glob('maps/*.yaml')),
        (os.path.join('share',package_name,'maps'),glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mountain',
    maintainer_email='mountain@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop = bogie_navigation.teleop:main",
            "nav2_cmd = bogie_navigation.navigation_command:main",
            "webcam = bogie_navigation.webcam:main",
            "qr_code = bogie_navigation.qr_code:main",
            "led = bogie_navigation.led:main",
            "cafe = bogie_navigation.cafe_night:main",
        ],
    },
)
