from setuptools import find_packages, setup

package_name = 'turtlebot3_custom_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itzsyboo',
    maintainer_email='itzsyboo@uw.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = turtlebot3_custom_control.control_node:main',
            'distance_control_node = turtlebot3_custom_control.distance_control_node:main',
        ],
    },
)
