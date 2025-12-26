from setuptools import setup

package_name = 'rc_ackermann_joy'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joy_to_ackermann.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='ssafy@example.com',
    description='Joystick to Twist mapping.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_ackermann = rc_ackermann_joy.joy_to_ackermann:main',
        ],
    },
)
