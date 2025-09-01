from setuptools import setup

package_name = 'formation_control'

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
    maintainer='finn',
    maintainer_email='finnpicotoli@gmail.com',
    description='Multi-drone 3D formation control system with centralized safety monitoring',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'formation_commander = formation_control.formation_commander:main',
            'safety_monitor = formation_control.safety_monitor:main',
            'emergency_controller = formation_control.emergency_controller:main',
            'drone_interface = formation_control.drone_interface:main',
            'formation_visualizer = formation_control.formation_visualizer:main',
            'swarm_takeoff_commander = formation_control.swarm_takeoff_commander:main',
        ],
    },
)