from setuptools import find_packages, setup

package_name = 'fly_control'

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
    maintainer='Dobrea Dan',
    maintainer_email='mdobrea@gmail.com',
    description='UAV control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'elrs = fly_control.elrs:main',
             'fly  = fly_control.uav_ctrl:main',
        ],
    },
)
