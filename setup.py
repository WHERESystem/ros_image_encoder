from setuptools import setup

package_name = 'ros_image_encoder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/image_converter_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='travis',
    maintainer_email='travis@todo.todo',
    description='ROS 2 image encoding converter node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'convert_image_encoding = ros_image_encoder.convert_image_encoding:main',
        ],
    },
)
