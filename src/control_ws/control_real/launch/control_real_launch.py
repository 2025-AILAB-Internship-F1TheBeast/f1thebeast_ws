from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_real',         # package : 어느 패키지에서 실행할지
            namespace='control',         # namespace : 노드의 네임스페이스. Ex) namespace='control', name='controller' 일때 노드의 이름은 /control/controller
            executable='control_real_node',  # executable : 실행할 노드의 이름. 주로 실행할 파일 이름을 적음.
            name='controller',
            output='screen',  # output : 노드의 출력 위치. 'screen'은 터미널에 출력됨.
            parameters= [PathJoinSubstitution([
                FindPackageShare('control_real'), 'config', 'controller.yaml'])
            ],
        )
    ])