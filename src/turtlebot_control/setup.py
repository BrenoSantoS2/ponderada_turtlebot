from setuptools import find_packages, setup

package_name = 'turtlebot_control'

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
    maintainer='breno',
    maintainer_email='breno.santos.ismart@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cli_control = turtlebot_control.cli_control:main",
            "video_server = turtlebot_control.video_server:main",
            "turtlebot_controller = turtlebot_control.turtlebot_controller:main",
        ],
    },
)
