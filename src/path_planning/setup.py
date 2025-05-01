from setuptools import find_packages, setup
import os

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.{package_name}'],
    package_dir={
        '': '.',
        f'{package_name}.{package_name}': f'{package_name}'
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include your data files
        (os.path.join('share', package_name, 'data'), 
            [os.path.join(package_name, 'data', 'test.pol')]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='kehillah',
    maintainer_email='kehillah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)