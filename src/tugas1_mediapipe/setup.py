from setuptools import find_packages, setup

package_name = 'tugas1_mediapipe'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hares',
    maintainer_email='haresyosuaa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe = tugas1_mediapipe.mediapipe:main',
            'camera_subscriber = tugas1_mediapipe.camera_subscriber:main'
        ],
    },
)
