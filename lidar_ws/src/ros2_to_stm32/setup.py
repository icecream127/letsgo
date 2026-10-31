from setuptools import find_packages, setup
from glob import glob

package_name = 'ros2_to_stm32'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['map_zones_config.json', 'map_zones_config_2.json']},
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hurricane',
    maintainer_email='hurricane@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_adapter = ros2_to_stm32.nav_adapter:main',
            'nav_cmd_gate = ros2_to_stm32.nav_cmd_gate:main',
        'pos_trans = ros2_to_stm32.trans:main',
            'info_trans = ros2_to_stm32.info_transmiting:main',
            'serial_node = ros2_to_stm32.serial:main',
            'map_node = ros2_to_stm32.map:main',
            'ui_node = ros2_to_stm32.ui:main',
        ],
    },
)
