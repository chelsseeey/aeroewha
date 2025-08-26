from setuptools import setup
package_name = 'offboard_control'  # ← 언더스코어

setup(
    name=package_name,
    version='0.0.1',
    packages=['offboard_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='ae',
    maintainer_email='ae@todo.todo',
    description='Offboard control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'offboard_node = offboard_control.offboard_node:main',
            'hover_node = offboard_control.hover_node:main',   
        ],
    },
)
