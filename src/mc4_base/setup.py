from setuptools import setup

package_name = 'mc4_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matrix Code Lab',
    maintainer_email='matrixcodelab@gmail.com',
    description='MC 4.0 base driver, fake hardware and fake lidar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mc4_base_node = mc4_base.base_node:main',
            'fake_lidar_node = mc4_base.fake_lidar_node:main',
        ],
    },
)
