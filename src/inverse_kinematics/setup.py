from setuptools import setup

package_name = 'inverse_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='parsa hedayati',
    maintainer_email='parsahd95@gmail.com',
    description='Leg kinematics ROS 2 node for quadruped robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematics_node = inverse_kinematics.inverse_kinematics_node:main',
            'movement_node = inverse_kinematics.movement_node:main',
            'command_publisher = inverse_kinematics.command_publisher:main'
        ],
    },
)
