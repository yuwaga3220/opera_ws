from setuptools import find_packages, setup

package_name = 'pure_pursuit_ugo'

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
    maintainer='hoge',
    maintainer_email='hoge@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_point_node = pure_pursuit_ugo.follow_point:main',
            'follow_point2_node = pure_pursuit_ugo.follow_point2:main',
            'follow_point_odom_node = pure_pursuit_ugo.follow_point_odom:main',
            
            'joy_to_cmdvel_node = pure_pursuit_ugo.joy_to_cmdvel:main',
            'sensor_subscriber_node = pure_pursuit_ugo.sensor_subscriber:main',
            
            'obstacle_stop_node = pure_pursuit_ugo.obstacle_stop:main',
            'mux_node = pure_pursuit_ugo.mux:main',
            
            'joint_sub_node = pure_pursuit_ugo.joint_sub:main',
            
            'ik_command_tester = pure_pursuit_ugo.tip_to_joint_angles:main',
            'joy_to_tipGoal_node = pure_pursuit_ugo.joy_to_tipGoal:main',
            'tipGoal_to_angles_node = pure_pursuit_ugo.tipGoal_to_angles:main',
            
            'joy_to_tipGoal_node2 = pure_pursuit_ugo.joy_to_tipGoal2:main',
            'tipGoal_to_angles_node2 = pure_pursuit_ugo.tipGoal_to_angles2:main',
            'swing_controller_node = pure_pursuit_ugo.swing_controller:main',
            'swing_alignment_checker_node = pure_pursuit_ugo.swing_alignment_checker:main',
            
            'tipGoal_to_angles_node3 = pure_pursuit_ugo.tipGoal_to_angles3:main',
            # フリーズ注意！！
            'joy_unified_controller_node = pure_pursuit_ugo.joy_unified_controller:main',
            
            'ControllCatNode = pure_pursuit_ugo.controll_cat:main',
            'ControllCat2Node = pure_pursuit_ugo.controll_cat2:main',
            
            # catで関節操作とタスク空間操作を統合したスクリプト（動かない）
            'UnifiedControllerWithZoom = pure_pursuit_ugo.unified_controller_with_zoom:main',

            # catでインチング制御にしようとしたが未完成
            'ControllCat3Node = pure_pursuit_ugo.controll_cat3:main',
            
            # catを走行させるだけのスクリプト（結局失敗）
            'CatController = pure_pursuit_ugo.cat_controller:main',
            
            
            # シミュレーション用
            
            # zx120の関節とカメラを制御するスクリプト    
            'Zx120Controller = pure_pursuit_ugo.zx120_controller:main',
            # zx120でタスク空間操作をするスクリプト
            'Zx120IkController = pure_pursuit_ugo.zx120_ik_controller:main',
            # catの関節とカメラを制御するスクリプト
            'Cat303crController = pure_pursuit_ugo.cat303cr_controller:main',
            # catでタスク空間操作をするスクリプト（動く）
            'Cat303crIkController = pure_pursuit_ugo.cat303cr_ik_controller:main',    

            'IproCameraNode = pure_pursuit_ugo.ipro_driver:main',
            'JointErrorMonitor = pure_pursuit_ugo.joint_error_monitor:main',
            'GrappleTrackingNode = pure_pursuit_ugo.grapple_tracking_node:main',
            'IproGridTrackingNode = pure_pursuit_ugo.ipro_grid_tracking:main',
            'IproGridTracking2Node = pure_pursuit_ugo.ipro_grid_tracking2:main',
            
            # 実機用
            'WitmotionNode = pure_pursuit_ugo.witmotion_node:main', # IMU -> RosTopic
            'WitmotionNode2 = pure_pursuit_ugo.witmotion_node2:main', # IMU*2 -> RosTopic

            'IproOnvifTrackingNode = pure_pursuit_ugo.ipro_onvif_tracking:main', # Unity -> Camera
            'IproOnvifTracking2Node = pure_pursuit_ugo.ipro_onvif_tracking2:main', # IMU*2 -> Camera
            'crosshair_node = pure_pursuit_ugo.crosshair_node:main',
            'mock_joint_states_pub = pure_pursuit_ugo.mock_joint_states_pub:main',
            ],
    },
)
