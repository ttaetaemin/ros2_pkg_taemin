from setuptools import setup

package_name = 'pinky_tmproject'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tm_centerline_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/points.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tm',
    maintainer_email='you@example.com',
    description='Topic-driven centerline navigation with RViz markers.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tm_centerline_nav = pinky_tmproject.tm_navigator:main',
        ],
    },
)

