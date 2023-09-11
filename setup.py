from setuptools import setup

package_name = 'camera_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cam_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sahil Panjwani',
    maintainer_email='panjwani_sahil@artc.a-star.edu.sg',
    description='ROS2 package to publish webcam/external camera stream as image data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_node = camera_publisher.cam_node:main',
        ],
    },
)
