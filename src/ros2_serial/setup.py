from setuptools import setup

package_name = 'ros2_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isopodin-time',
    maintainer_email='ax90l0tl@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = ros2_serial.publisher_member_function:main',
                'serial_node = ros2_serial.serial_node:main',
                ],
    },
)
