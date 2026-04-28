from setuptools import setup
import os
from glob import glob

package_name = 'quadrotor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='losfansdepalomino@proton.me',
    description='Quadrotor UAV mathematical simulator — IRS Tarea 3',
    license='MIT',
    entry_points={
        'console_scripts': [
            'simulator = quadrotor_sim.simulator_node:main',
            'trajectory_runner = quadrotor_sim.trajectory_node:main',
        ],
    },
)
