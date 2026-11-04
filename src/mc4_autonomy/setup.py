from glob import glob

from setuptools import setup

package_name = 'mc4_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matrix Code Lab',
    maintainer_email='matrixcodelab@gmail.com',
    description='Autonomous missions for the MC 4.0 using Nav2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_patrol = mc4_autonomy.waypoint_patrol:main',
        ],
    },
)
