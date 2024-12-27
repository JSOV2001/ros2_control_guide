# Guide for implementing ros2_control in URDF and Gazebo    
**Warning:** This guide is specifically about ROS2's *Humble* variant to be clear. However, this guide probably works the same for all ROS2 distributions.

## Table of Content
 1. Pre-requirements
 2. Installation
 3. Structure
 4. Implementation
	 4.1. Part 1: URDF file
	 4.2. Part 2: Yaml file
	 4.3. Part 3: Launch file
	 4.4. Part 4: Basic node
 5. Appendix

## Pre-Requirements

 - Basic understanding of URDF design
 - URDF's well-computed inertia matrices
 - URDF's well-selected frictions

In case that the reader doesn't meet the requirements, then read the appendix for more information.

## Installation
Please, install the following ROS2 packages through Ubuntu terminal:

    sudo apt-install ros-humble-ros2-control*
    sudo apt-install ros-humble-gazebo-ros2-control*
    sudo apt install ros-humble-control*
    sudo apt install ros-humble-resources*
 
It's worth noting that all of these are required to use ros2_control properly

## Structure
In simple works, ros2_control is a package that allows the developer to move a robot within Gazebo, and even in real-life with the proper setup.

Now, ros2_control has three components:
 
 - **ROS2 controllers.** These're the set of algorithms that effectively moves the robot.

 - **Controller manager.** It's a package that allows the developer to *adapt* the chosen ROS2 controller to the robot's joints.
 
 - **Hardware interface.** These're the joints that *can* actually be controlled by any ros2_controller. Why's that? Because moving your robot within a simulation isn't an URDF's inherent ability, henceforth the developer has to specify it themself in code.

When it comes to using ros2_control, this is the general step-by-step: 

1. *Choose the ros2_controller according to the robot's unique needs.*  
    
    **Example 1:** In a robot manipulator, it's possible to control each individual joint's position within Gazebo by using the *position_controllers/JointGroupPositionController*

	**Example 2:** In a *car-like* mobile robot, it's possible to to move it within Gazebo by using the *bicycle_steering_controller/BicycleSteeringController*

2. *Adapt the algorithm to the robot's joints in some file* (we'll see that in the next section).

    **Example 1:** In a robot manipulator, the developer tells the controller manager which joints can be controlled by *position_controllers/JointGroupPositionController*.
    
	**Example 2:** In a *car-like* mobile robot, the developer tells the controller manager which joints can be controlled by *bicycle_steering_controller/BicycleSteeringController*

3. *Specify the robot's joints which can actually be controlled by any ros2_controller in some file* (we'll see that in the next section).

Remember, this is the *conceptual* structure on how ros2_control works. Now let's see how this is actually implemented.

## Implementation
In order to implement ros2_control, the developer needs the next three files:

1. **URDF.** In relation to ros2_control, URDF's purpose is to define can joints can actually be controlled by any ros2_controller. 

	Remember, this is just a definition, not the "run" of all this.

2. **Yaml.** In relation to ros2_control, yaml's purpose is to define the controller manager, so the chosen ros2_controller is adapted to the robot's unique needs. 

	The same with URDF: this is just another definition, not the "run" of all this.

3. **Launch.** In relation to ros2_control, launch's purpose is to automates the "run" process of the URDF and Yaml files with a command.

It's worth noting that these files are just to get URDF ready to be used with ros2_control. For moving the robot, a separated code must be developed.

From now on, a detailed example of how to integrate ros2_control in a 6-DOF manipulator will be explored.  Although this process may seem tedious and repetitive at the beginning, it brings a whole lot of advantages when it comes to prototyping robots, which is the ROS2's main purpose.

The full project can checked in this repository: https://github.com/JSOV2001/centauri.git, this code is still a work-in-progress, so any future changes will be updated in due time.

### Part 1: URDF file

For the URDF design, the *xacro* format will be used, as it allows the divide the URDF into easy readable files.  

**Note:** Now, this is a xacro file exclusively to the ros2_control part, not the robot structure itself.

	<?xml version="1.0"?>
	<robot xmlns:xacro="http://www.ros.org/wiki/xacro" >
    <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
            <robotNamespace>centauri</robotNamespace>
	           <parameters>$(find 		centauri)/config/joint_group_position_controller.yaml</parameters>
            <parameters>$(find centauri)/config/joint_trajectory_controller.yaml</parameters>
        </plugin>
    </gazebo>

    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>

        <joint name="joint1">
            <command_interface name="position">
                <param name="min">-1.5708</param>
                <param name="max">1.5708</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>  
            </joint>
            
        <joint name="joint2">
            <command_interface name="position">
                <param name="min">-1.5708</param>
                <param name="max">1.5708</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>  
        </joint>

        <joint name="joint3">
            <command_interface name="position">
                <param name="min">-1.5708</param>
                <param name="max">1.5708</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>  
        </joint>

        <joint name="joint4">
            <command_interface name="position">
                <param name="min">-1.5708</param>
                <param name="max">0.0</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>  
        </joint>

        <joint name="joint5">
            <command_interface name="position">
                <param name="min">-1.5708</param>
                <param name="max">0.0</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>  
        </joint>

        <joint name="joint6">
            <command_interface name="position">
                <param name="min">-1.5708</param>
                <param name="max">0.0</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <param name="initial_position">0.0</param>  
        </joint>
    </ros2_control>
</robot >

### Part 2: Yaml file 
Next, a yaml file will be presented. 

This yaml adapts the *position_controllers/JointGroupPositionController* to control each joint's position:
	
	# Define the ros2_controller to be used
    controller_manager:
	  ros__parameters:
	    update_rate: 100
		# This controller is necessary to read the joints' variables
	    joint_state_broadcaster:
	      type: joint_state_broadcaster/JointStateBroadcaster
		# Chosen controller to control each joint position
	    joint_group_position_controller:
	      type: position_controllers/JointGroupPositionController
	
	# Adapt the controller to the 6-DOF manipulator
	joint_group_position_controller:
	  ros__parameters:
		# Define which joints will be controlled
	    joints:
	      - joint1
	      - joint2
	      - joint3
	      - joint4
	      - joint5
	      - joint6
	
		# Each joint' position can be controlled
	    command_interfaces:
	      - position
	
		# Each joint' position can be read
	    state_interfaces:
	      - position

### Part 3: Launch file
Next, a launch file will be presented. 

This launch automates the running the *position_controllers/JointGroupPositionController* integrated in the robot manipulator:

    # Libraries for file handling
    import os
    from ament_index_python import get_package_share_directory
    import xacro
    # Libraries for node launching
    from launch.substitutions import LaunchConfiguration
    from launch.actions import DeclareLaunchArgument
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    from launch import LaunchDescription
    
    # URDF's filepath
    pkg_filepath = get_package_share_directory("centauri_description")
    xacro_filepath = os.path.join(pkg_filepath, "description", "centauri.urdf.xacro")
    robot_description_file = xacro.process_file(xacro_filepath)
    
    # ROS2 arguments for launch configutation
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    def generate_launch_description():
        use_sim_time_arg = DeclareLaunchArgument(
            "use_sim_time", 
            default_value= "true", 
            description= "Use sim time if true"
        )
    
        use_ros2_control_arg = DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'
        )

	    robot_state_publisher_node = Node(
	        package= "robot_state_publisher",
	        executable= "robot_state_publisher",
	        parameters=[
	            {
	                "robot_description": robot_description_file.toxml(),
	                "use_sim_time": use_sim_time
	            }
	        ]
	    )

	    gazebo_cmd = ExecuteProcess(
	        cmd = ["gazebo", "-s", "libgazebo_ros_factory.so"]
	    )
	    
	    gazebo_spawner_node = Node(
	        package = "gazebo_ros",
	        executable = "spawn_entity.py",
	        arguments = [
	            '-topic', 'robot_description', 
	            '-entity', 'centauri'
	        ],
	    )
	    
		# Activates the controller_manager described in yaml file
	    ros2_control_node = Node(
	        package= "controller_manager",
	        executable= "ros2_control_node",
	        remappings=[
	            ("~/robot_description", "robot_description")
	        ]
	    )
		
		# Activates specifically the joint_state_broadcaster described in yaml file
	    jsb_spawner_node = Node(
	        package="controller_manager",
	        executable="spawner",
	        arguments= ["joint_state_broadcaster"]
	    )
	    
	    # Activates specifically the joint_group_position_controller described in yaml file
	    jgpc_spawner_node = Node(
	        package="controller_manager",
	        executable="spawner",
	        arguments= ["joint_group_position_controller"]
	    )
	    
	    nodes_to_run = [
	        use_sim_time_arg,
	        use_ros2_control_arg,
	        robot_state_publisher_node,
	        gazebo_cmd, 
	        gazebo_spawner_node,
	        ros2_control_node,
	        jsb_spawner_node,
	        jgpc_spawner_node
	    ]
	    return LaunchDescription(nodes_to_run)
    
 ### Part 4: Basic node
Next, a basic node will be presented. 

This node is a typical publisher that adds 0.1 radians to each joint, until it finally reaches to 15°:

    enter #!/usr/bin/env python3
	# ROS libraries
	import rclpy
	from rclpy.node import Node
	from std_msgs.msg import Float64MultiArray
	# Python libraries
	import math

	class JointGroupPositionPublisher(Node):
	    def __init__(self):
	        super().__init__('joint_group_position_publisher')

	        self.joint_initial_angle = math.radians(0)
	        self.joint_final_angle = math.radians(15)
	        self.joint_current_angle = 0.0
	        self.increment_in_joint = 0.0

	        self.angle_publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)
	         
	        self.angle_msg = Float64MultiArray()
	        self.interpolation_timer = self.create_timer(0.1, self.publish_joint_position)
	    
	    def publish_joint_position(self):
		    # Add radians to joint group
	        self.joint_current_angle = self.joint_initial_angle + self.increment_in_joint

	        if(self.joint_current_angle >= self.joint_final_angle):
	            self.joint_current_angle = self.joint_final_angle
			
			# Publish current joint group position
	        self.angle_msg.data = [
	             self.joint_current_angle,
	             self.joint_current_angle,
	             self.joint_current_angle,
	             self.joint_current_angle,
	             self.joint_current_angle,
	             self.joint_current_angle
	        ]
	        self.angle_publisher.publish(self.angle_msg)
	        
	        self.increment_in_joint += 0.01

	def main(args=None):
	    rclpy.init(args=args)
	    node = JointGroupPositionPublisher()
	    try:
	        rclpy.spin(node)
	    except KeyboardInterrupt:
	        node.destroy_node()
	        rclpy.try_shutdown()

	if __name__ == "__main__":
	    main()

## Appendix
Here are material to study the pre-requirement concepts:
 
 - Basic understanding of launch files: https://youtu.be/xJ3WAs8GndA?si=FwYxho-53tvdnjz8}

 - Basic understanding of yaml files: https://youtu.be/wY8MrBGVxYA?si=Dfan_ZK1H1aVEauI

 - Basic understanding of URDF design:  
	 - https://youtu.be/CwdbsvcpOHM?si=M9Wi3KgdUE8NVK1x
	 - https://youtu.be/BcjHyhV0kIs?si=WGnV4h1VIKXjo9hu
	 - https://youtu.be/QyvHhY4Y_Y8?si=Txs_jRZJirs6QUHp
 
 - Basic understanding of custom URDF creation:  https://youtube.com/playlist?list=PL6TtDG40DrFpPf7OQagUNSW8dZMAvs5F0&si=8XfpnN0vtaETIE33
 
 - Computation of URDF's inertia matrix: https://github.com/gstavrinos/calc-inertia
 
 - Selection of Gazebo Classic's frictions: https://classic.gazebosim.org/tutorials/?tut=ros_urdf, although Gazebo Classic will lose support by January 2025.

## REFERENCES
 - https://control.ros.org/humble/doc/getting_started/getting_started.html
 - https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html
 - https://youtu.be/4QKsDf1c4hc?si=PAFanuoA0zBLt10v
 - https://youtu.be/4VVrTCnxvSw?si=iNvol8ZCpBbznvc2
