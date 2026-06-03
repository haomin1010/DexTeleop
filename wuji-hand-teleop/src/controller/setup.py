from setuptools import setup, find_packages

package_name = 'controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.24.0',
    ],
    zip_safe=True,
    maintainer='Wuji Robotics',
    maintainer_email='dev@wuji.com',
    description='Unified controller node - state machine with mode switching support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tianji_arm_controller = controller.tianji_arm_node:main',
            'tianji_sdk_executor = controller.tianji_sdk_executor_node:main',
            'tianji_joint_state_bridge = controller.tianji_joint_state_bridge_node:main',
            'tianji_mujoco_viewer = controller.tianji_mujoco_viewer_node:main',
            'tianji_tracker_sim_viz = controller.tianji_tracker_sim_viz_node:main',
            'wujihand_controller = controller.wujihand_node:main',
            'wujihand_direct_controller = controller.wujihand_direct_node:main',
        ],
    },
)
