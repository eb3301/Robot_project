from setuptools import setup
import os
from glob import glob

package_name = 'py_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')), 
    ],
    install_requires=['setuptools', 'detect_interfaces'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='Python Service Example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = py_srvcli.service_member_function:main',
            'client = py_srvcli.client_member_function:main',
            'detection_service = py_srvcli.detection_service:main', 
            'detection_client=py_srvcli.detection_client:main',
        ],
    },
)
