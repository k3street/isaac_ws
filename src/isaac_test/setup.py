from setuptools import find_packages, setup

package_name = 'isaac_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_control_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimate',
    maintainer_email='kimate@todo.todo',
    description='Isaac Sim Camera Data Subscriber and Processor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = isaac_test.camera_subscriber:main',
            'camera_controller = isaac_test.camera_controller:main',
            'isaac_camera_controlled = isaac_test.isaac_camera_controlled:main',
        ],
    },
)
