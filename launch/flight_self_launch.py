from launch import LaunchDescription
from launch_ros.actions import  Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # flight_id_set_arg = DeclareLaunchArgument(
    #     'flight_id',default_value=TextSubstitution(text='0')
    # )

    config = os.path.join(
        get_package_share_directory('flight'),
        'config',
        'flight_config.yaml'
    )

    return LaunchDescription(
        [
            #flight_id_set_arg,
            Node(
                package='flight',
                executable='flight',
                name='flightNode',
                #parameters=[config]
                parameters=[
                    {
                        'flight_id':"0",
                        'com_number':'/dev/ttyTHS0'
                    }
                ]
            ),
        ]
    )