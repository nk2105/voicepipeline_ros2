import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur_whisper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'srdf'), glob(os.path.join('srdf', '*'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ninad Kulkarni',
    maintainer_email='ninad.kulkarni@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander = ur_whisper.commander:main',
            'transcriber = ur_whisper.transcriber:main',
            'commander_gpt = ur_whisper.commander_gpt:main',
            'spot_commander = ur_whisper.spot_commander:main',
            'ur_moveit_motion = ur_whisper.ur_moveit_motion:main',
        ],
    },
)
