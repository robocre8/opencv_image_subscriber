from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'opencv_image_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'image_raw_subscriber = opencv_image_subscriber.image_raw_subscriber:main',
            'image_compressed_subscriber = opencv_image_subscriber.image_compressed_subscriber:main',
        ],
    },
)
