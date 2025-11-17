from setuptools import setup

package_name = 'armrobot_teach'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teach_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghost',
    maintainer_email='bouhachem.ahmed@outlook.com',
    description='Teach and replay named joint poses using MoveIt.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teach_node = armrobot_teach.teach_node:main',
        ],
    },
)
