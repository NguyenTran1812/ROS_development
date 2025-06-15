from setuptools import setup

package_name = 'px4_sim_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='NguyenTran',
    maintainer_email='vannguyenag2000@gmail.com',
    description='Replay flight data into PX4 sim topics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'real_to_sim_bridge = px4_sim_bridge.real_to_sim_bridge:main'
        ],
    },
)

