from setuptools import find_packages, setup

package_name = 'lab_quaternion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/lab_quaternion/launch', ['launch/sort_world.launch.py']),
        ('share/lab_quaternion/worlds', ['worlds/sort_world.sdf']),
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
            'trajectory_from_csv = lab_quaternion.trajectory_from_csv:main',
            'gen3lite_pymoveit2 = lab_quaternion.gen3lite_pymoveit2:main',
            'sort_cubes = lab_quaternion.sort_cubes:main',
            'test_arm_movement = lab_quaternion.test_arm_movement:main',
            'go_vertical = lab_quaternion.go_vertical:main',
            'go_to_position = lab_quaternion.go_to_position:main',
            'control_gripper = lab_quaternion.control_gripper:main',
            'control_phy_gripper = lab_quaternion.control_phy_gripper:main',
            'final_mover = lab_quaternion.final_move:main'

        ],
    },
)
