from setuptools import find_packages, setup

package_name = 'my_controller'

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
    maintainer='andreeabruchental',
    maintainer_email='andreeabruchental@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_controller.my_first_node:main",
            "publisher = my_controller.publisher:main",
            "subscriber = my_controller.subscriber:main",
            "subscriber_with_rosbag = my_controller.subscriber_with_rosbag:main"
        ],
    },
)
