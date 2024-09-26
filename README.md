## Guide for implementing *ros2_control* in an URDF and Gazebo

*Note:* As the title suggests, this guide only focus on using ros2_control in Gazebo, not in real-world robots.

## Section 0: Pre-requirements

**Warning 1:** I assume you got the following concepts:
 
 - *Basic* understanding of **URDF design**. If not, you can check:  
	 - https://youtu.be/CwdbsvcpOHM?si=M9Wi3KgdUE8NVK1x
	 - https://youtu.be/BcjHyhV0kIs?si=WGnV4h1VIKXjo9hu
	 - https://youtu.be/QyvHhY4Y_Y8?si=Txs_jRZJirs6QUHp
 
 - *Basic* understanding of **custom URDF creation**. If not, you can check:  https://youtube.com/playlist?list=PL6TtDG40DrFpPf7OQagUNSW8dZMAvs5F0&si=8XfpnN0vtaETIE33

 - *Basic* understanding of **yaml files**. If not, you can check: https://youtu.be/wY8MrBGVxYA?si=Dfan_ZK1H1aVEauI
 
 - *Basic* understanding of **launch files**. If not, you can check: https://youtu.be/xJ3WAs8GndA?si=FwYxho-53tvdnjz8
 
**Warning 2:** I assume your URDF's inertia matrices and frictions are sufficiently well-computed. If not, you can use the following tool and information: 

 - For URDF's inertia matrix: https://github.com/gstavrinos/calc-inertia
 - For Gazebo's frictions: https://classic.gazebosim.org/tutorials/?tut=ros_urdf
 
**Warning 3:** I'm talking about the ROS2's *Humble* variant specifically. However, I'm pretty sure this guide works the same for all ROS2 distributions, I just prefer to make the acclaration beforehand.

**Warning 4:** Please, install the following ROS2 packages in your Ubuntu systems:

    sudo apt-install ros-humble-ros2-control*
    sudo apt-install ros-humble-gazebo-ros2-control*
    sudo apt install ros-humble-control*
    sudo apt install ros-humble-resources*
 
With that out of the way, Let's go!

## Section 1: Structure of *ros2_control*

In simple words, ros2_control is just a package that allow you to move a robot in Gazebo, and even in real-life with the proper setup.

This package is difficult because, as every useful tool, it needs some extensive configuration. But, don't worry, this guide is here to guide you.

### Parts of ros2_control

Now, ros2_control has three components:
 
 - **ROS2 controllers.** These're the set of algorithms that effectively moves your robot in Gazebo.

 - **Controller manager.** It's a package that allows you to *adapt* you chosen ROS2 controller to your robot's joints.
 
 - **Hardware interface.** This is the actuator/joint that *can* actually be controlled. Why do we need to define that? Well, because moving your robot in simulation isn't an URDF's inherent ability, henceforth you have to specify it yourself. 

### Use of ros2_control

So, this is the general step-by-step when it comes to using ros2_control: 

1. You choose your ros2_controller according to your robot's unique needs.  
    
    For example, in a robot manipulator, you can control each individual joint's position, velocity and effort in the simulation.

	Another example, if you define that your wheeled mobile robot is an *car-like* type, then you can use the *bicycle_steering_controller/BicycleSteeringController* to move it in Gazebo.

2. You adapt the algorithm to your robot's joints in some file (we'll see that in the next section).

	For example, you tell the controller manager which joints in your robot manipulator are about to be controlled.

3. You need to specify that your robot's joints can actually be controlled in some file (we'll see that in the next section).

Remember, this is the *conceptual* structure on how ros2_control works. Now let's see how this is actually implemented.

*Note:* In other documentation, you'll the terms **resources manager**, it's just the collection of all hardware interfaces, because each harward interface is an actuator by itself. But that's just something conceptual.

## Section 2: Implementation of *ros2_control*

In order to implement ros2_control, you need three files:

1. **URDF.** When it comes to ros2_control, URDF's purpose is to define can joints can actually be controlled by ros2_controller. Remember, this is just a definition, not the "run" of all this.

2. **Yaml file.** When it comes to ros2_control, yaml's purpose is to define the controller manager, so your chosen ros2_controller is adapted to your robot's unique needs. Remember, this is just a definition, not the "run" of all this.

3. **Launch file.** When it comes to ros2_control, the launch file makes the automation to run all of your URDF and Yaml files in just one easy command.

Now that we know the purpose of this file. I'll give a detailed example of how I'd apply ros2_control to a 6-DOF manipulator named Centauri. 

*Note 1:* Throughout the code you'll see two terms: *state_interfaces* and *command_interfaces*. Just keep in mind this:

 - command_interfaces are the variables we can control  
 - state_interfaces are the variables we can read

*Note 2:* You can check the full code in here: https://github.com/JSOV2001/centauri.git

*Note 3:* Please, remember that the robots' code and this guide are works in progress, so don't worry if you see any changes in the future. I'll update everything in due time.

### Phase 1: ros2_control and URDF

As personal advice, I recommend you to always use the *xacro* format for your URDF, as it allows the divide your URDF in easy digestible files.  

*Note:* Now, this is a xacro file exclusively to the ros2_control part. In this robot (and probably in your robot), you'll have to upload with the rest of your URDF.

	<?xml version="1.0"?><!-- Hardware interface for robot's controller -->
	<ros2_control  name="GazeboSystem"  type="system">
		<hardware>
			<!-- Hardware interface will be Gazebo -->
			<plugin>gazebo_ros2_control/GazeboSystem</plugin>
		</hardware>

		<!-- Joint 1 can be controlled -->
		<joint  name="joint1">
			<!-- Joint 1's control variable is position -->
			<command_interface  name="position">
				<param  name="min">-1.5708</param>
				<param  name="max">1.5708</param>
			</command_interface>
			<!-- Joint 1's position can be read-->
			<state_interface  name="position"/>
			<!-- Joint 1's velocity can be read-->
			<state_interface  name="velocity"/>
			<param  name="initial_position">0.0</param>
			</joint>
	
		<!-- The same goes for every joint -->

		<joint  name="joint2">
			<command_interface  name="position">
				<param  name="min">-1.5708</param>
				<param  name="max">1.5708</param>
			</command_interface>
			<state_interface  name="position"/>
			<state_interface  name="velocity"/>
			<param  name="initial_position">0.0</param>
		</joint>
		
		<joint  name="joint3">
			<command_interface  name="position">
				<param  name="min">-1.5708</param>
				<param  name="max">1.5708</param>
			</command_interface>
			<state_interface  name="position"/>
			<state_interface  name="velocity"/>
			<param  name="initial_position">0.0</param>
		</joint>
		
		<joint  name="joint4">
			<command_interface  name="position">
				<param  name="min">-1.5708</param>
				<param  name="max">1.5708</param>
			</command_interface>
			<state_interface  name="position"/>
			<state_interface  name="velocity"/>
			<param  name="initial_position">0.0</param>
		</joint>

		<joint  name="joint5">
			<command_interface  name="position">
				<param  name="min">-1.5708</param>
				<param  name="max">1.5708</param>
			</command_interface>
			<state_interface  name="position"/>
			<state_interface  name="velocity"/>
			<param  name="initial_position">0.0</param>
		</joint>

		<joint  name="joint6">
			<command_interface  name="position">
				<param  name="min">-1.5708</param>
				<param  name="max">1.5708</param>
			</command_interface>
			<state_interface  name="position"/>
			<state_interface  name="velocity"/>
			<param  name="initial_position">0.0</param>
		</joint>
	</ros2_control>

	<!-- Load robot's controller -->
	<gazebo>
		<plugin  filename="libgazebo_ros2_control.so"  name="gazebo_ros2_control">
		<robotNamespace>centauri</robotNamespace>
		<parameters>$(find centauri)/config/controllers.yaml</parameters>
	</plugin>

	</gazebo>
	</robot >

### Phase 2: ros2_control and yaml file

As personal advice, I recommend you to always use the *xacro* format for your URDF, as it allows the divide your URDF in easy digestible files.  

*Note:* Take into account that this yaml file uses the *position_controllers/JointGroupPositionController* to control each joint's angular position.
	
	# Define the ros2_controller to be used
    controller_manager:
	  ros__parameters:
	    update_rate: 100
		# This controller is necessary to read the joints' variables
	    joint_state_broadcaster:
	      type: joint_state_broadcaster/JointStateBroadcaster
	      
		# Chosen controller to control each joint position
	    joint_group_position_controller: # Chosen controller's name
	      type: position_controllers/JointGroupPositionController
	
	# Adapt the chosen controller to the 6-DOF manipulator
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

### Phase 3: ros2_control and launch file

Remember, you could do this manually, but the launch is a great to saving time, so you can focus on more important things in your projects. 

The launch file I'll show you it's just a standard launch file, what's important is the node to be launched.

    # Libraries for file handling
	import  os
	from  ament_index_python  import  get_package_share_directory
	import  xacro

	# Libraries for node launching
	from  launch_ros.actions  import  Node
	from  launch.actions  import  ExecuteProcess
	from  launch  import  LaunchDescription
	from  launch.actions  import  DeclareLaunchArgument
	from  launch.substitutions  import  LaunchConfiguration
	
	pkg_filepath  =  get_package_share_directory("centauri")
	
	xacro_filepath  =  os.path.join(pkg_filepath, "description", "centauri.urdf.xacro")

	robot_description_file  =  xacro.process_file(xacro_filepath)
	
	use_sim_time  =  LaunchConfiguration("use_sim_time")
	use_ros2_control  =  LaunchConfiguration('use_ros2_control')
	
	def  generate_launch_description():

	use_sim_time_arg  =  DeclareLaunchArgument(
		"use_sim_time",
		default_value=  "true",
		description=  "Use sim time if true"
	)

	use_ros2_control_arg  =  DeclareLaunchArgument(
		'use_ros2_control',
		default_value='true',
		description='Use ros2_control if true'
	)

	robot_state_publisher_node  =  Node(
		package=  "robot_state_publisher",
		executable=  "robot_state_publisher",
		parameters=[
			{
				"robot_description": robot_description_file.toxml(),
				"use_sim_time": use_sim_time
			}
		]
	)
	
	gazebo_cmd  =  ExecuteProcess(cmd  = ["gazebo", "-s", "libgazebo_ros_factory.so"])

	gazebo_spawner_node  =  Node(
		package  =  "gazebo_ros",
		executable  =  "spawn_entity.py",
		arguments  = [
			'-topic', 'robot_description',
			'-entity', 'centauri'
		],
	)
	
	# Upload controller
    ros2_control_node = Node(
        package= "controller_manager",
        executable= "ros2_control_node",
        parameters= [robot_description_config, controller_filepath]
    )
	
	# Run joint_state_broadcaster controller
    jsb_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments= ["joint_state_broadcaster"]
    )
	
	# Run joint_group_position_controller controller
    jpc_spawner_node = Node(
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
	    jpc_spawner_node
	]
    return LaunchDescription(nodes_to_run)
    
 Things I'd like you to understand:

 - You need to upload your Yaml and URDF files to the launch file.
 
 - We uses two nodes from *controller_manager* package: *ros2_control_node* and *spawner*.
 
 - The *ros2_control_node* proccess your configurations in URDF and yaml files, so the your customized ros2_controllers are available to run.
 
 - The *spawner* run your customized ros2_controllers.
 
 ### Phase 4: ros2_control and node

Next, I'll show a simple code to just move joints:. This is just a regular publisher node in the file "joint_group_position_publisher.py":
 

    #!/usr/bin/env python3
	# ROS2 libraries
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
	        
			self.controller_topic  =  "/joint_group_position_controller/commands"

			self.controller_pub  =  self.create_publisher(Float64MultiArray, self.controller_topic, 10)

			self.controller_msg  =  Float64MultiArray()
				        self.angle_publisher = self.create_publisher(Float64MultiArray, "/joint_group_position_controller/commands", 10)
	         
	        self.interpolation_timer = self.create_timer(0.1, self.publish_joint_position)
	    
	    def publish_joint_position(self):
	        self.joint_current_angle = self.joint_initial_angle + self.increment_in_joint

	        if(self.joint_current_angle >= self.joint_final_angle):
	            self.joint_current_angle = self.joint_final_angle
			
			# The order is the same described in yaml file
	        self.controller_msg.data = [
	             self.joint_current_angle, # Joint 1
	             self.joint_current_angle, # Joint 2
	             self.joint_current_angle, # Joint 3
	             self.joint_current_angle, # Joint 4
	             self.joint_current_angle, # Joint 5
	             self.joint_current_angle  # Joint 6
	        ]
	        self.controller_pub.publish(self.controller_msg)
	        
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

 Things I'd like you to understand:

 - Every ros2_controller have a lot of topics. 
 
 - Usually, the topic for controlling joints is the *command topic*, and it has the name "commands" in it.
 
 - The commands topic's name sintaxis usually is: "/**Controller's name**/commands". Always remember to put your controller's name.
 
 - You can run this node with the following command, just like any regular node:

    ros2 run centauri joint_group_position_publisher.py
 
 - Each *command topic* has its message type and it's up to the programmer to find it.

## Final Words

Well, that's all, folks!

I know all of this may seem too tedious at the beginning... but I hope you'll eventually realize this is kind of a repetitive proccess and brings a whole lot a advantages when it comes to prototyping robots, which is the main purpose of ROS2.

## REFERENCES

 1. https://control.ros.org/humble/doc/getting_started/getting_started.html
 2. https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html
 3. https://youtu.be/4QKsDf1c4hc?si=PAFanuoA0zBLt10v
 4. https://youtu.be/4VVrTCnxvSw?si=iNvol8ZCpBbznvc2
