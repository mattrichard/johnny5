from setuptools import setup

package_name = 'johnny5_teleop'

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
    maintainer='matt',
    maintainer_email='matthew.w.richard@gmail.com',
    description='Teleoperation for the Johnny 5 robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop = johnny5_teleop.joy_teleop:main'
        ],
    },
)
