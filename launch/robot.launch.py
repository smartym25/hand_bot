import os
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robotXacroName= "hand"

    namePackage="hand_bot"

    modelFilePath="description/robot.urdf.xacro"

    #worldFilePath="worlds/empty.world"

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFilePath)

    #pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFilePath)

    robotDescription= xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"),
                                                                       "launch", "gz_sim.launch.py"))
    
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={"gz_args" : ["-r -v -v4 empty.sdf"], "on_exit_shutdown": "true"}.items())

    spawnModelNodeGazebo = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robotXacroName,
            "-topic", "robot_description"
        ],
        output="screen",
    )

    nodeRobotStatePublisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robotDescription,
        "use_sim_time": True}
        ],
        output="screen",
    )

    bridge= os.path.join(get_package_share_directory(namePackage), "parameters", "bridge_parameters.yaml")

    start_gazebo_ros_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[bridge],
        output= "screen",
    )

    LaunchDescriptionObject= LaunchDescription()

    LaunchDescriptionObject.add_action(gazeboLaunch)

    LaunchDescriptionObject.add_action(spawnModelNodeGazebo)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)
    LaunchDescriptionObject.add_action(start_gazebo_ros_bridge)

    return LaunchDescriptionObject
