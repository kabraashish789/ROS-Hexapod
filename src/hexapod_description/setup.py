from setuptools import setup
import os
from glob import glob

package_name = 'hexapod_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/.world'))
       
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
               'motor_data1=hexapod_description.controller1:main',
               "motor_data2=hexapod_description.controller2:main",
               "motor_data3=hexapod_description.controllerSpot:main",
               "motor_data4=hexapod_description.controllerTrigo:main",
               'patrol_action_server_exe = hexapod_description.patrol_action_server:main',
               

        ],
    },
)
