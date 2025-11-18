from setuptools import setup, find_packages

package_name = 'pinky_tmpatrol'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/waypoints.yaml',
            'config/nav2_overrides.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/wallpatrol.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='you@example.com',
    description='Wall-hugging waypoint patrol for Pinky using Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = pinky_tmpatrol.patrol_node:main',
            'click2yaml  = pinky_tmpatrol.click2yaml:main',
        ],
    },
)
