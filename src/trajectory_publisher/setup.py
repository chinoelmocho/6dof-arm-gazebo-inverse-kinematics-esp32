from setuptools import find_packages, setup

package_name = 'trajectory_publisher'

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
    description='Genera y publica trayectorias cartesianas troceadas en /trajectory_chunks para el brazo de 6 GDL con micro-ROS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher_node = trajectory_publisher.trajectory_publisher_node:main',
        ],
    },
)
