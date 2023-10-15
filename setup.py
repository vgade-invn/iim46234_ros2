from setuptools import find_packages, setup

package_name = 'iim46234_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishal',
    maintainer_email='vishal.gade@tdk.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iim46234_node = iim46234_ros2.iim46234_node:main'
        ],
    },
)
