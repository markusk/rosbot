from setuptools import setup
import os
from glob import glob

package_name = 'testbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Markus',
    maintainer_email='github@direcs.de',
    description='My ROS 2 test node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'testnode = testbot.testnode:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
)
