from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    spawner = Node(
        package="panda_vrep",
        executable="vrep_control",
        name="vrep_control",
        output="log"
    )

    gui = LaunchConfiguration("gui")
    bool = None
    if not IfCondition(gui):
        bool = True
    else:
        bool = False

    vrep=ExecuteProcess(
        cmd=["LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH QT_QPA_PLATFORM_PLUGIN_PATH=${COPPELIASIM_ROOT} $COPPELIASIM_ROOT_DIR/coppeliaSim.sh" + (" -h" if bool else "")],
        shell=True,
        output="log",
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
            'gui',
            default_value="true",
            description='GUI'),

            vrep, 
            spawner
        ]

    )