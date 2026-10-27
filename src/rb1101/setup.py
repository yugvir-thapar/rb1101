from setuptools import find_packages, setup

package_name = 'rb1101'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        include=['std_msgs', 'sensor_msgs', 'nav_msgs', 'rb1101'],
        exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marmot',
    maintainer_email='marmot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_following = rb1101.wall_following:main',
        ],
    },
)
