from setuptools import find_packages, setup

package_name = 'turtlebot_move'

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
    maintainer='gixstudent',
    maintainer_email='rkumar23@uw.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
       		'final_code = turtlebot_move.final_code:main',
            'gen3lite_pymoveit2 = 516_final.gen3lite_pymoveit2:main',
        ],
    },
)
