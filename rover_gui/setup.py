from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'pages', 'static'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ui'), glob('ui/*')),
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
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
