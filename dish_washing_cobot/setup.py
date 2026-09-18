from setuptools import find_packages, setup

package_name = 'dish_washing_cobot'

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
    maintainer='seb000423',
    maintainer_email='seb000423@gmail.com',
    description='Doosan M0609 협동로봇으로 접시·그릇·컵을 형상별 궤적으로 세척하는 ROS2 패키지',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_node = dish_washing_cobot.main_node:main',
            'plate_test = nodes.plate_test_node:main',
            
        ],
    },
)
