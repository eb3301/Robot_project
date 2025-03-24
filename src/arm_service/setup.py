from setuptools import find_packages, setup

package_name = 'arm_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='dharas@kth.se',
    description='Arm serrvice',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_service = arm_service.arm_service:main',
            'arm_client = arm_service.arm_client:main', 
            'test = arm_service.testrun:main',
        ],
    },
)
