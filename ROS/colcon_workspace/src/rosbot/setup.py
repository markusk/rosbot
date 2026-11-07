from glob import glob
import os

from setuptools import setup

package_name = 'rosbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Markus',
    maintainer_email='github@direcs.de',
    description='My "rosbot" ROS 2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'LEDPWMNode = rosbot.LEDPWMNode:main',
            'MotorControllerNode = rosbot.MotorControllerNode:main',
            'TeleopKeyboardNode = rosbot.TeleopKeyboardNode:main',
            'TeleopJoystickNode = rosbot.TeleopJoystickNode:main',
            'TeleopArbitratorNode = rosbot.TeleopArbitratorNode:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('lib', package_name), glob('scripts/*.py')),
    ],
)
