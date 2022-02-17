from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    spawner = Node(
        package="panda_vrep",
        executable="vrep_control",
        name="vrep_control",
        output="log"
    )

    gui = LaunchConfiguration("gui")
    vrep_com = "LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH QT_QPA_PLATFORM_PLUGIN_PATH=${COPPELIASIM_ROOT} $COPPELIASIM_ROOT_DIR/coppeliaSim.sh"
    tmp = PythonExpression(['"',vrep_com, '" + " -h" if "', gui, '" == "false" else "', vrep_com,'"'])
    vrep=ExecuteProcess(
        cmd=[tmp],
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