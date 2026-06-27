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
        ('share/' + package_name, ['ax12_control/adam.rviz']),
        ('share/' + package_name + '/launch', [
            'launch/visualizar_marcha.launch.py',
            'launch/controle_manual.launch.py',
        ]),
    ],
    # Faz o(s) YAML(s) de marcha viajarem junto com o módulo na instalação,
    # para que o caminho padrão (otimizada.yaml ao lado do send_gait) seja
    # encontrado também depois do colcon build.
    package_data={package_name: ['*.yaml', '*.rviz']},
    include_package_data=True,
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
            'visualizar_marcha = ax12_control.visualizar_marcha:main',
            'gait_bridge = ax12_control.gait_bridge:main',
            'passo_slider = ax12_control.passo_slider:main',
            'controle_manual = ax12_control.controle_manual:main',
        ],
    },
)
