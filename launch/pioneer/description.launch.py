#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
import xml.dom.minidom

import xacro

def generate_launch_description():

    use_sim_time     = LaunchConfiguration('use_sim_time',  default='false')
    namespace_arg    = DeclareLaunchArgument('namespace',   default_value=TextSubstitution(text=''))
    model_name_arg   = DeclareLaunchArgument('model_name',  default_value=TextSubstitution(text='robot0'))
    robot_arg        = DeclareLaunchArgument('robot',       default_value=TextSubstitution(text='pioneer3dx'))

    this_pgk = 'tuw2_launches'
    config_pgk_dir = get_package_share_directory(this_pgk)



    def create_robot_description(context): 
        models_dir = os.path.join(get_package_share_directory('tuw_gazebo_models'), 'models')
        xacro_file = os.path.join(models_dir, context.launch_configurations['robot'], 'main.xacro')
        xml_file = os.path.join(config_pgk_dir, 'config', 'robot_description', context.launch_configurations['robot'], 'robot_description.xml')
        assert os.path.exists(xacro_file), "The main.xacro doesnt exist in "+str(xacro_file)
        robot_description_config = xacro.process_file(xacro_file, 
            mappings={  "namespace": context.launch_configurations['namespace'], 
                        "models_dir": models_dir})
        robot_desc = robot_description_config.toxml()
        dom = xml.dom.minidom.parseString(robot_desc)
        f = open(xml_file, "w")   ## for debugging only
        f.write(dom.toprettyxml())
        f.close()
        return [SetLaunchConfiguration('robot_desc', robot_desc)]
        
        
    # Not working: loading the description without xacro
    # def read_robot_description(context): 
    #    xml_file = os.path.join(config_pgk_dir, 'config', 'robot_description', context.launch_configurations['robot'], 'robot_description.xml')
    #    with open(xml_file, 'r') as infp:
    #       robot_desc = infp.read()
    #    robot_desc = str(robot_desc)
    #    return [SetLaunchConfiguration('robot_desc', robot_desc)]
    #
    #create_robot_description_arg = OpaqueFunction(function=read_robot_description)
    
    create_robot_description_arg = OpaqueFunction(function=create_robot_description)

    return LaunchDescription([
        namespace_arg,
        robot_arg,
        create_robot_description_arg,
        model_name_arg,
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="robot_joint_state_publisher",
            parameters=[{
				"zeros.right_wheel_joint": 1.0,
                "use_sim_time": use_sim_time,
                "rate": 10}]),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=[LaunchConfiguration('namespace')],
            # this works but I will not use it at the moment
            # remappings=[
            #    ("/tf", "/r0/tf"),
            #    ("/tf_static", "/r0/tf_static")
            #],
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": LaunchConfiguration('robot_desc')}],
            output="screen"),
    ])
