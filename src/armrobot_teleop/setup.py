from setuptools import setup

package_name = 'armrobot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghost',
    maintainer_email='bouhachem.ahmed@outlook.com',
    description='Joystick teleoperation nodes for the armrobot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_teleop = armrobot_teleop.joystick_teleop:main',
        ],
    },
)
