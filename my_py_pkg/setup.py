from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('launch/*.launch.xml')),
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
            "py_node = my_py_pkg.my_first_node:main",
            "py_oop_node = my_py_pkg.my_first_oop_node:main",
            "py_pub_node = my_py_pkg.py_simple_pub:main",
            "py_sub_node = my_py_pkg.py_simple_sub:main",
            "my_teleop = my_py_pkg.teleop:main",
        ],
    },
)
