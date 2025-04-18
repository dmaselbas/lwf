from setuptools import find_packages, setup

package_name = 'lwf_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pandas'],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='dmaselbas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = lwf_imu.imu_publisher:main'
        ],
    },
)
