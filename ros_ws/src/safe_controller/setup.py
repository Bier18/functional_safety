from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'safe_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'log_files'), glob('log_files/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_sender = safe_controller.TrajectorySender:main',
            'collision_publisher = safe_controller.CollisionPublisher:main',
            'safe_controller = safe_controller.SafeController:main',
            'logger = safe_controller.Logger:main'
        ],
    },
)
