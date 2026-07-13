import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import shutil


def resolve_yolo_device():
    forced = os.environ.get('SKKU_YOLO_DEVICE')
    if forced:
        return forced
    return 'cuda:0' if shutil.which('nvidia-smi') else 'cpu'

def generate_launch_description():
    yolo_device = resolve_yolo_device()
    # 각 카메라에 맞는 모델 경로
    camera_pkg_share = get_package_share_directory('camera_pkg')
    cam0_model_path = os.path.join(camera_pkg_share, 'model', 'best.pt')
    lane_image_dir = os.path.join(camera_pkg_share, 'lib', 'lane2')
    rviz_config = os.path.join(get_package_share_directory('launch_pkg'), 'rviz', 'general.rviz')

    return LaunchDescription([
        # #################### CAMERA1(LANE) ######################
        
        Node(
            package='camera_pkg',
            executable='image',  # 실행할 노드 파일명
            name='cam0',
            namespace='cam0',
            parameters=[
                {'data_source': 'camera'},
                # {'data_source': 'camera'},  # camera, video, image 선택
                {'cam_num': 4},
                {'img_dir': lane_image_dir},
                {'pub_topic': '/cam0/image_raw'},
                {'window_name': 'Raw 0'},
                {'show_image': False},
                # {'timer_period': 0.03},  # float형으로, fps조절 
                {'timer_period': 0.03},
            ]
        ),
        
        #################### CAMERA1 YOLO SEG ######################
        SetEnvironmentVariable('CUDA_VISIBLE_DEVICES', '0'),
        Node(
            package='camera_pkg',
            executable='yolo_seg',  
            name='yolo_seg0',
            namespace='cam0',
            output='screen',
            parameters=[
                # cpu or cuda:0
                {'device': yolo_device},
                {'model_path': cam0_model_path},
                {'threshold': 0.5}
            ],
            remappings=[
                ('image_raw', '/cam0/image_raw'),  
                ('detections', '/cam0/detections'),
                ('seg_vis', '/cam0/seg_vis')
            ]
        ),

        #################### LANE DETECT ######################
        Node(
            package='camera_pkg',
            executable='lane',  
            name='lane_detector',
            namespace='cam0',
            parameters=[
                {'camera_topic': '/cam0/image_raw'},
                {'detection_topic': '/cam0/detections'}
            ],
            output='screen'
        ),
        
        #################### MOTION ######################
        Node(
            package='decision_making_pkg',  
            executable='motion',  
            name='motion_node',
            output='screen'
        ),
        
        #################### CONTROL ######################
        #Node(
            #package='control_pkg',  
            #executable='control',  
            #name='control_node',
            #output='screen'
        #),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
    ])
