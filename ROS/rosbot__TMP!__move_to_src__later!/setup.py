from setuptools import setup

package_name = 'rosbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Markus Knapp',
    maintainer_email='ros@direcs.de',
    description='The rosbot package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = rosbot.my_python_script:main'
        ],
    },
)
