from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'smbus2'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Sensor interfaces package for sailbot2425',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radio_comm = sensors.radio_comm_node:main',
            'gps = sensors.gps:main',
            'arduino_interface = sensors.arduino_interface:main',
            'rudder_control = sensors.rudder_control_node:main',
        ],
    },
)