## *Introduction*

The motive of this project is to implement an autonomous warehouse management system which takes order from the users and process the orders effectively based on the priority of the packages.

The theme given to us is 'Vargi-Bots' which means separation of objects based on a category using robots. This theme uses two robotic arms named UR5_1 and UR5_2. This theme mainly focuses on Industry 4.0 where the warehouses are automated to reduce manpower, optimise the processes, carry out the tasks in a smart way and make continuous improvements. Orders like Medicines, Food and Clothes are sent to the MQTT server from different places. 
One robotic arm named UR5_1 will pick the respective packages from a shelf according to the incoming orders and place them on a conveyor belt. Another robotic arm named UR5_2 will collect these packages from the conveyor belt and places them into the respective bins for shipment of the packages to the respective places. Orders received are of different priorities like high priority, medium priority and low priority for medicine, food and clothes respectively. An algorithm is used to pick the packages based on priority to ship the high priority packages effectively. When the packages are sent out from the warehouse, email alerts are sent to the users notifying them about the status of the package.


All the information related to the order, dispatch and shipment of each package is published on a google sheet and in a user-friendly website for easy accessing and understanding. These spreadsheets log every data which can be used for verification in future. The website gives real time information about the happenings in the warehouse with visual data like bar graphs and line graphs. Even a map is included to show from which place the order is generated and the status of the particular package.

#### *ROS, Gazebo and Moveit*

*Let's see how we use ROS,  Gazebo and Moveit platforms in our theme*

##### *ROS*

ROS (Robot Operating System) is an open source platform, which has many libraries and tools to help software developers to create robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The version of ROS we have used is Melodic which supports industrial applications. In ROS we have different nodes which are used to communicate between the MQTT server and the UR5 robots. 

##### *Gazebo*

In this project ROS and gazebo play a major role. ROS is the central processing unit of this project whereas Gazebo is the simulation environment which shows the demonstration of this project. These platforms are used for two-third of the complete objective of the project with the help of multiple ROS packages. ROS packages include multiple nodes (python scripts), action files, message files, config files. 

Gazebo offers the ability to simulate robots and other complex structures accurately and efficiently in almost any type of environment. We have used Gazebo 9.0.0 which can support almost all the packages of ROS melodic. In Gazebo we could simulate UR5 arms to know how it behaves in the environment according to our algorithm. 

##### *Moveit*

MoveIt is the most widely used software for manipulation and has been used on over 150 real-time robots. It is released under the terms of the BSD license, and thus free for industrial, commercial, and research use. By incorporating the latest advances in motion planning, manipulation, 3D perception, kinematics, control and navigation, MoveIt is state of the art software for mobile manipulation.

#### *Implementation Video*

We have embedded a youtube video which will demonstrate the complete implementation of this theme. We will also discuss all the implementation techniques and API's used in this theme at the next section.



<iframe width="1080" height="580" src="https://youtube.com/embed/E_6gIIUICUw" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

