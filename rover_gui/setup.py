from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_gui'

setup(
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('resource'), glob('ui/*')),
        (os.path.join('resource'), glob('ui/*')),
        (os.path.join('resource'), glob('images/*')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('resource'), glob('resource/*')),
        (os.path.join('share/ament_index/resource_index/packages'), glob('resource/' + package_name))
    ],
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'pages', 'static'],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathan',
    maintainer_email='nathan.dumoulin11@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "main_gui = rover_gui.main:main"
        ],
    },
)
