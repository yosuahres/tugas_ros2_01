from setuptools import setup

package_name = 'tugas1_mediapipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'tugas1_msgs'],
    zip_safe=True,
    maintainer='hares',
    maintainer_email='haresyosuaa@gmail.com',
    description='Mediapipe ROS 2 integration',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe = tugas1_mediapipe.mediapipe:main',
            # contoh: 'node_name = tugas1_mediapipe.node:main',
        ],
    },
)
