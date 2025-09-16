from setuptools import find_packages, setup
import os

package_name = 'mbmadpi'

# Collect all files in launch directory
launch_files = []
for root, dirs, files in os.walk(os.path.join(package_name, '..', 'launch')):
    for file in files:
        launch_files.append(os.path.join(root, file))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='traenkle',
    maintainer_email='frank.traenkle@hs-heilbronn.de',
    description='MAD76 Python-Only Package',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rcnode = mbmadpi.rcnode:main',
            'ctrlnode = mbmadpi.ctrlnode:main',
            'safectrlnode = mbmadpi.safectrlnode:main',
        ],
    },
)
