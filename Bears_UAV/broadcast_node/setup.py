from setuptools import find_packages, setup

package_name = 'broadcast_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dobrea Dan-Marius',
    maintainer_email='mdobrea@gmail.com',
    description='broadcast node code',
    license='3-Clause BSD license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcast = broadcast_node.broadcast:main',
        ],
    },
)
