
from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'decision_making_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bogdan',
    maintainer_email='urxel@student.kit.edu',
    description='Decision making algorithm for pedestrian crossing simulation - urxel master thesis.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_making_node = decision_making_package.decision_making_node:main',
            'custom_decision_making_node = decision_making_package.custom_decision_making_package:main',
        ],
    },
)
