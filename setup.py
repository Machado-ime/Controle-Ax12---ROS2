from setuptools import find_packages, setup

package_name = 'ax12_control'

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
    maintainer='Fernando Machado',
    maintainer_email='machado.fernando@ime.eb.br',
    description='Controle de servomotores Dynamixel AX-12 via ROS 2 para robo bipede.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ax12_controller = ax12_control.ax12_controller:main',
            'send_gait = ax12_control.send_gait:main',
            'ax12_monitor = ax12_control.ax12_monitor:main',
        ],
    },
)
