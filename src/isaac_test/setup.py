from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isaac_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimate',
    maintainer_email='kimate@todo.todo',
    description='Isaac Sim Camera Data Subscriber and Processor with LLM Provider Support',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_camera_controller = isaac_test.llm_camera_controller:main',
            'llm_camera_controller_simple = isaac_test.llm_camera_controller_simple:main',
        ],
    },
)
