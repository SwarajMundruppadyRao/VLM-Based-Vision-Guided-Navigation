from setuptools import setup
from glob import glob
import os

package_name = 'vision_ocr'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Swaraj Mundruppadyrao',
    maintainer_email='swaraj@example.com',
    description='OCR node for text detection in vision-guided navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ocr_node = vision_ocr.ocr_node:main',
        ],
    },
)
