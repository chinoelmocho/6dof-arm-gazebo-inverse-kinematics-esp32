from setuptools import find_packages, setup

package_name = 'ik_bridge'

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
    maintainer='chino',
    maintainer_email='chino@todo.todo',
    description=(
        'Puente de prueba: punto deseado -> /trajectory_chunks (ESP32) -> '
        '/ik_result_chunks -> arm_controller, siguiendo /joint_states en '
        'tiempo real y notificando saltos de articulacion.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_bridge_node = ik_bridge.ik_bridge_node:main',
            'fk_node = ik_bridge.fk_node:main',
            'send_target = ik_bridge.send_target:main',
        ],
    },
)
