from setuptools import setup # Removed find_packages as it's not used for this specific setup.
import os

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.{package_name}'], # e.g., ['path_planning', 'path_planning.path_planning']
    package_dir={
        '': '.',
        # This maps the Python package 'path_planning.path_planning'
        # to be sourced from the 'path_planning' directory relative to setup.py
        f'{package_name}.{package_name}': package_name
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line installs test.pol to share/path_planning/data/test.pol.
        # You can keep it if other non-Python parts of your system need to find it there.
        (os.path.join('share', package_name, 'data'),
            [os.path.join(package_name, 'data', 'test.pol')]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=False,  # Important for package_data to work reliably with __file__
    maintainer='kehillah',
    maintainer_email='kehillah@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    package_data={
        # This ensures 'data/test.pol' is installed alongside your Python modules.
        # The key f'{package_name}.{package_name}' (i.e., 'path_planning.path_planning')
        # matches the Python package containing leg.py.
        # The path 'data/test.pol' is relative to the source directory of that package
        # (which is 'sailbot2425/src/path_planning/path_planning/').
        f'{package_name}.{package_name}': ['data/test.pol']
    }
)
