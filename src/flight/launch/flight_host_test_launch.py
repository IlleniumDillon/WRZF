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
                executable='flight',
                name='flightNode0',
                parameters=[
                    {
                        'flight_id':"0"
                    }
                ]
            ),
            Node(
                package='flight',
                executable='flight',
                name='flightNode1',
                parameters=[
                    {
                        'flight_id':"1"
                    }
                ]
            ),
            Node(
                package='flight',
                executable='flight',
                name='flightNode2',
                parameters=[
                    {
                        'flight_id':"2"
                    }
                ]
            ),
            Node(
                package='flight',
                executable='flight',
                name='flightNode3',
                parameters=[
                    {
                        'flight_id':"3"
                    }
                ]
            ),
            Node(
                package='flight',
                executable='flight',
                name='flightNode4',
                parameters=[
                    {
                        'flight_id':"4"
                    }
                ]
            ),
        ]
    )