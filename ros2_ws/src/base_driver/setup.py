from setuptools import setup

package_name = 'base_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toybrick',
    maintainer_email='toybrick@example.com',
    description='莱斯特麦轮底盘 ROS 2 驱动节点：/cmd_vel -> AT+MT_SPWM，AT+MTENCODER? -> /odom + TF',
    license='MIT',
    entry_points={
        'console_scripts': [
            'chassis_node = base_driver.chassis_node:main',
        ],
    },
)
