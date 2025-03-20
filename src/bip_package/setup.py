from setuptools import find_packages, setup

package_name = 'bip_package'

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
    maintainer='ros',
    maintainer_email='Janusz.Jakubiak@pwr.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = bip_package.my_talker:main',
            'my_talker = bip_package.publisher_member_function:main',
            'imu_simulator = bip_package.imu_simulator:main',
            'tf_simulator = bip_package.rollpitch_simulator:main',
        ],
    },
)
