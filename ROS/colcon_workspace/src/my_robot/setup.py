from setuptools import setup
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # <-- Wichtig für Module
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Markus',
    maintainer_email='markus@example.com',
    description='Mein ROS 2 Paket',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot.my_node:main',  # <-- Korrekte Modulreferenz
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), glob('scripts/*.py')),  # <-- Skripte richtig kopieren
    ],
)
