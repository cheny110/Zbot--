
# Zbot 底盘驱动 ROS API 

## 双目款

- 节点
    /zbot3_drive
- 接受话题
    |cmd_vel|   速度控制|
- 发布话题
    |/battery |  电量信息|
    |/odom_raw|  初始里程计信息|
    |/imu     |  惯导传感器(六轴陀螺仪)话题 |
    |/zbot3_drive/zbot3_state |  zbot3 状态话题（控制模式，急停状态）|

- 提供服务
    |/zbot3_drive/emergency_control | 急停控制       |


- 参数
    | /zbot3_drive/Kp | PID比例系数 |
    | /zbot3_drive/Ki | PID 积分系数 |

- 节点
    /zbot3_state_publisher
- 接受话题
    | /joint_states| 处理的关节状态列表|
- 发布话题 
    | /robot_States| 机器人URDF模型状态|

- 发布TF
    | /base_footprint->/base_link | 底盘实体投影到底盘实体的坐标变换 |
    | /base_link -> camera_link    | 底盘实体到双目相机实体坐标变换  |
    | /base_link -> laser         | 底盘实体到雷达中心的坐标变换 |
    | /base_link -> *_wheel_link  | 底盘实体到各轮子的坐标变换   |

## 双目相机款

- 驱动相关节点
    /camera/camera_nodelet_manager
    /camera/camera_rgb
    /camera/depth_metric
    /camera/depth_metric_rect
    /camera/depth_points
    /camera/depth_rectify_depth
    /camera/depth_registered_hw_metric_rect
    /camera/depth_registered_metric
    /camera/depth_registered_rectify_depth
    /camera/driver
    /camera/points_xyzrgb_hw_registered
    /camera/rgb_rectify_color
    /camera_base_link
    /camera_base_link1
    /camera_base_link2
    /camera_base_link3

- 发布的图像话题
    /camera/camera_nodelet_manager/bond
    /camera/camera_rgb/parameter_descriptions
    /camera/camera_rgb/parameter_updates
    /camera/depth/camera_info
    /camera/depth/image
    /camera/depth/image/compressed
    /camera/depth/image/compressed/parameter_descriptions
    /camera/depth/image/compressed/parameter_updates
    /camera/depth/image/compressedDepth
    /camera/depth/image/compressedDepth/parameter_descriptions
    /camera/depth/image/compressedDepth/parameter_updates
    /camera/depth/image/theora
    /camera/depth/image/theora/parameter_descriptions
    /camera/depth/image/theora/parameter_updates
    /camera/depth/image_raw
    /camera/depth/image_raw/compressed
    /camera/depth/image_raw/compressed/parameter_descriptions
    /camera/depth/image_raw/compressed/parameter_updates
    /camera/depth/image_raw/compressedDepth
    /camera/depth/image_raw/compressedDepth/parameter_descriptions
    /camera/depth/image_raw/compressedDepth/parameter_updates
    /camera/depth/image_raw/theora
    /camera/depth/image_raw/theora/parameter_descriptions
    /camera/depth/image_raw/theora/parameter_updates
    /camera/depth/image_rect
    /camera/depth/image_rect/compressed
    /camera/depth/image_rect/compressed/parameter_descriptions
    /camera/depth/image_rect/compressed/parameter_updates
    /camera/depth/image_rect/compressedDepth
    /camera/depth/image_rect/compressedDepth/parameter_descriptions
    /camera/depth/image_rect/compressedDepth/parameter_updates
    /camera/depth/image_rect/theora
    /camera/depth/image_rect/theora/parameter_descriptions
    /camera/depth/image_rect/theora/parameter_updates
    /camera/depth/image_rect_raw
    /camera/depth/image_rect_raw/compressed
    /camera/depth/image_rect_raw/compressed/parameter_descriptions
    /camera/depth/image_rect_raw/compressed/parameter_updates
    /camera/depth/image_rect_raw/compressedDepth
    /camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
    /camera/depth/image_rect_raw/compressedDepth/parameter_updates
    /camera/depth/image_rect_raw/theora
    /camera/depth/image_rect_raw/theora/parameter_descriptions
    /camera/depth/image_rect_raw/theora/parameter_updates
    /camera/depth/points
    /camera/depth_rectify_depth/parameter_descriptions
    /camera/depth_rectify_depth/parameter_updates
    /camera/depth_registered/camera_info
    /camera/depth_registered/hw_registered/image_rect
    /camera/depth_registered/hw_registered/image_rect/compressed
    /camera/depth_registered/hw_registered/image_rect/compressed/parameter_descriptions
    /camera/depth_registered/hw_registered/image_rect/compressed/parameter_updates
    /camera/depth_registered/hw_registered/image_rect/compressedDepth
    /camera/depth_registered/hw_registered/image_rect/compressedDepth/parameter_descriptions
    /camera/depth_registered/hw_registered/image_rect/compressedDepth/parameter_updates
    /camera/depth_registered/hw_registered/image_rect/theora
    /camera/depth_registered/hw_registered/image_rect/theora/parameter_descriptions
    /camera/depth_registered/hw_registered/image_rect/theora/parameter_updates
    /camera/depth_registered/hw_registered/image_rect_raw
    /camera/depth_registered/hw_registered/image_rect_raw/compressed
    /camera/depth_registered/hw_registered/image_rect_raw/compressed/parameter_descriptions
    /camera/depth_registered/hw_registered/image_rect_raw/compressed/parameter_updates
    /camera/depth_registered/hw_registered/image_rect_raw/compressedDepth
    /camera/depth_registered/hw_registered/image_rect_raw/compressedDepth/parameter_descriptions
    /camera/depth_registered/hw_registered/image_rect_raw/compressedDepth/parameter_updates
    /camera/depth_registered/hw_registered/image_rect_raw/theora
    /camera/depth_registered/hw_registered/image_rect_raw/theora/parameter_descriptions
    /camera/depth_registered/hw_registered/image_rect_raw/theora/parameter_updates
    /camera/depth_registered/image
    /camera/depth_registered/image/compressed
    /camera/depth_registered/image/compressed/parameter_descriptions
    /camera/depth_registered/image/compressed/parameter_updates
    /camera/depth_registered/image/compressedDepth
    /camera/depth_registered/image/compressedDepth/parameter_descriptions
    /camera/depth_registered/image/compressedDepth/parameter_updates
    /camera/depth_registered/image/theora
    /camera/depth_registered/image/theora/parameter_descriptions
    /camera/depth_registered/image/theora/parameter_updates
    /camera/depth_registered/image_raw
    /camera/depth_registered/image_raw/compressed
    /camera/depth_registered/image_raw/compressed/parameter_descriptions
    /camera/depth_registered/image_raw/compressed/parameter_updates
    /camera/depth_registered/image_raw/compressedDepth
    /camera/depth_registered/image_raw/compressedDepth/parameter_descriptions
    /camera/depth_registered/image_raw/compressedDepth/parameter_updates
    /camera/depth_registered/image_raw/theora
    /camera/depth_registered/image_raw/theora/parameter_descriptions
    /camera/depth_registered/image_raw/theora/parameter_updates
    /camera/depth_registered/points
    /camera/depth_registered_rectify_depth/parameter_descriptions
    /camera/depth_registered_rectify_depth/parameter_updates
    /camera/driver/parameter_descriptions
    /camera/driver/parameter_updates
    /camera/ir/camera_info
    /camera/ir/image
    /camera/ir/image/compressed
    /camera/ir/image/compressed/parameter_descriptions
    /camera/ir/image/compressed/parameter_updates
    /camera/ir/image/compressedDepth
    /camera/ir/image/compressedDepth/parameter_descriptions
    /camera/ir/image/compressedDepth/parameter_updates
    /camera/ir/image/theora
    /camera/ir/image/theora/parameter_descriptions
    /camera/ir/image/theora/parameter_updates
    /camera/projector/camera_info
    /camera/rgb/camera_info
    /camera/rgb/image_raw
    /camera/rgb/image_raw/compressed
    /camera/rgb/image_raw/compressed/parameter_descriptions
    /camera/rgb/image_raw/compressed/parameter_updates
    /camera/rgb/image_raw/compressedDepth
    /camera/rgb/image_raw/compressedDepth/parameter_descriptions
    /camera/rgb/image_raw/compressedDepth/parameter_updates
    /camera/rgb/image_raw/theora
    /camera/rgb/image_raw/theora/parameter_descriptions
    /camera/rgb/image_raw/theora/parameter_updates
    /camera/rgb/image_rect_color
    /camera/rgb/image_rect_color/compressed
    /camera/rgb/image_rect_color/compressed/parameter_descriptions
    /camera/rgb/image_rect_color/compressed/parameter_updates
    /camera/rgb/image_rect_color/compressedDepth
    /camera/rgb/image_rect_color/compressedDepth/parameter_descriptions
    /camera/rgb/image_rect_color/compressedDepth/parameter_updates
    /camera/rgb/image_rect_color/theora
    /camera/rgb/image_rect_color/theora/parameter_descriptions
    /camera/rgb/image_rect_color/theora/parameter_updates
    /camera/rgb_rectify_color/parameter_descriptions
    /camera/rgb_rectify_color/parameter_updates



## 带KATA 机器人款

- 节点
    /kata_drive

- 发布话题
    | /kata/joints_angle | KATA 各关节角度 |
    | /kata/kata/status  |  KATA 部分状态信息（基坐标系位姿，就绪状态，气泵状态）|

- 发布服务
    | /kata/axis_home    | KATA复位  |
    |/kata/go_camera_pose | KATA 移动到预设拍照位姿 |
    | /kata/go_zero       | KATA 各关节回零   |
    | /kata/pose_control  | KATA 移动控制     |
    | /kata/pump_control  | KATA 气泵控制    |
    | /kata/save_camera_pose | KATA保存当前位姿为预设拍照位姿 |
    | /kata/set_speed       | KATA设置默认运行速度    |
    | /kata/set_tool_offset  | KATA设置工具坐标系偏移 |

- 节点
    /kata_cam  

- 发布话题
    | /kata_cam/

- 发布服务
    | /kata_cam/start_capture  | 开启视频流捕获 |
    | /kata_cam/stop_capture   | 停止视频流捕获  |

