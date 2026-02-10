from setuptools import setup
import os
from glob import glob

package_name = 'image_processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.publisher', f'{package_name}.subscriber'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arda',
    maintainer_email='ardacihan7452@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_processor.publisher.publisher_node:main',
            'image_listener = image_processor.subscriber.subscriber_node:main',
        ],
    },
)