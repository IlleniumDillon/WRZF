from launch import LaunchDescription
from launch_ros.actions import  Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    flight_id_set_arg = DeclareLaunchArgument(
        'flight_id',default_value=TextSubstitution(text='0')
    )
    return LaunchDescription(
        [
            flight_id_set_arg,
            Node(
                package='flight',
                executable='yolo',
                name='YoloNode',
                parameters=[
                    {
                        'imgsize_min':"1.0",
                        'imgsize_max':"1.2",
                        'conf_min':"1.3",
                        'conf_max':"1.0"
                    }
                ]
            ),
            Node(
                package='flight',
                executable='flight',
                name='flightNode',
                parameters=[
                    {
                        'flight_id':"0",
                    }
                ]
            ),
        ]
    )