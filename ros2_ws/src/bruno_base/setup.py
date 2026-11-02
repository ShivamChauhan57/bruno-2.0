from setuptools import find_packages, setup

package_name = 'bruno_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bruno',
    maintainer_email='shivamchauhan3957@gmail.com',
    description='ROS 2 bridge node for Bruno\'s ESP32 base controller (motor commands + sonar telemetry over serial)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bruno_base_node = bruno_base.bruno_base_node:main',
        ],
    },
)
