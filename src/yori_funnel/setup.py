from setuptools import setup
from setuptools import find_packages

package_name = 'yori_funnel'

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
    maintainer='isopodin-time',
    maintainer_email='ax90l0tl@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'funnel_sub = yori_funnel.yori_funnel_sub:main'
        ],
    },
)
