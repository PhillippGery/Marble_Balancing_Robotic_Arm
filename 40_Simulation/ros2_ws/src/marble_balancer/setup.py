from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'marble_balancer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.sdf') + glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gery',
    maintainer_email='pgery@purdue.edu',
    description='LQR marble balancing controller for UR robot arm via MoveIt2',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'lqr_controller = marble_balancer.lqr_controller_node:main',
            'tcp_executor   = marble_balancer.tcp_executor_node:main',
            'velocity_bridge = marble_balancer.velocity_bridge:main',
            'marble_servo_controller = marble_balancer.marble_servo_controller:main',
            'go_to_pose = marble_balancer.go_to_pose:main',
        ],
    },
)
