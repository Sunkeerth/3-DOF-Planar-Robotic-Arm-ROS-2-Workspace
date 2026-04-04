from setuptools import find_packages, setup

package_name = 'planar_arm_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suneerth',
    maintainer_email='suneerth@todo.todo',
    description='FK and IK nodes for 3-DOF planar arm',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fk_solver   = planar_arm_kinematics.fk_solver:main',
            'ik_solver   = planar_arm_kinematics.ik_solver:main',
            'test_ik_client = planar_arm_kinematics.test_ik_client:main',
        ],
    },
)
