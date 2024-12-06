from setuptools import find_packages, setup

package_name = 'whill_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/whill.launch.py']),
        ('share/' + package_name, ['launch/whill_state_publisher.launch.py']),
        ('share/' + package_name, ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='George Mandokoro',
    maintainer_email='george.mandokoro@whill.inc',
    description='This package boot WHILL Model CR2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
