from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wuji_glove_input_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('wuji_glove_input_py/config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
        'pyyaml>=6.0',
        'wuji-sdk>=0.10.0',
    ],
    zip_safe=True,
    maintainer='Wuji Tech',
    maintainer_email='support@wuji.tech',
    description='Wuji glove data conversion node for Wuji Hand teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wuji_glove_input = wuji_glove_input_py.wuji_glove_input_node:main',
        ],
    },
)
