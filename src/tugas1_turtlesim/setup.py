from setuptools import find_packages, setup

package_name = 'tugas1_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tugas1_msgs', 'cv_bridge'],
    zip_safe=True,
    maintainer='mob',
    maintainer_email='fernando142311@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle = tugas1_turtlesim.turtlesim:main',
        ],
    },
)
