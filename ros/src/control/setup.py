from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/run.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ziad',
    maintainer_email='ziad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = nodes.IMUNode:main',
            'depth_node = nodes.DepthNode:main',
            'joystick_channel_node = nodes.JoystickNode:main',
            'gui_streamer = nodes.GUIStreamingNode:main',
            'pwm_client_node = nodes.PWMNode:main',
        ],
    },
)
