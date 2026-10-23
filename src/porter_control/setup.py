from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'porter_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arda AYDIN',
    maintainer_email='aydinnard@gmail.com',
    description=('Colour detection, following behaviour and demo scenario '
                 'nodes for the Porter robot.'),
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'color_detector = porter_control.color_detector:main',
            'follow_controller = porter_control.follow_controller:main',
            'person_walker = porter_control.person_walker:main',
        ],
    },
)
