from setuptools import find_packages, setup

package_name = ('manipulator_servo_driver')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='stanislav.svediroh@vut.cz',
    description='Interface for mks servo',
    license='MIT',
    entry_points={
        'console_scripts': [
            "mks_interface = manipulator_servo_driver.driver:main",
        ],
    },
)
