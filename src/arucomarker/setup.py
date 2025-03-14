from setuptools import find_packages, setup

package_name = 'arucomarker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where="src"),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/MarkerPose.msg', 'msg/MarkerPoseArray.msg', 'msg/MarkerDetect.msg', 'msg/MarkerDetectArray.msg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haryeong',
    maintainer_email='haryeong@todo.todo',
    description='AruCo marker detection package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MarkerDetect = arucomarker.MarkerDetect:main'
        ],
    },
)