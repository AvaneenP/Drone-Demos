# Freyja Drone Demos

This repo contains two demos: controlling the x-y-z movement of the drone using your keyboard as well as having the drone hover above a moving ground reference (in this example our drone will hover over a moving TurtleBot3).

The drone that we will be using throughout these demos is a DJI Flamewheel running a Pixhawk controller.

Prior to starting these demos, make sure you complete the [Freyja tutorials](https://github.com/less-lab-uva/flame-wheel-beginner-guide). This also means you are familiar with using the VICON system.

The ROS Desktop uses ZSH rather than bash so commands have been changed for ZSH. You replace these commands by changing zsh to bash

## Setting up VICON

On your VICON Desktop, complete the following steps:
1. Make sure the VICON camera system is up and running
2. Run ifconfig in the Ubuntu terminal. Ensure that the IP address for this machine is 192.168.3.3 (or something similar)
3. Make sure you can ping the ROS Desktop. In Ubuntu terminal run:
```
export ROS_IP = 192.168.3.3
export ROS_HOSTNAME = 192.168.3.3
export ROS_MASTER_URI = http://192.168.3.2:11311
```
4. cd catkin_ws/
5. source devel/setup.bash. *After this step, set up the ROS Desktop and come back to complete step #6*
6. roslaunch vicon_bridge vicon.launch (make sure roscore is already running on the ROS Desktop)
----
On your ROS Desktop, complete the following steps:
1. Run ifconfig in the Ubuntu terminal. Ensure that the IP address for this machine is 192.168.3.2 (or something similar)
2. Make sure you can ping the VICON Desktop
3. In Ubuntu terminal run:
```
export ROS_IP = 192.168.3.2
export ROS_HOSTNAME = 192.168.3.2
export ROS_MASTER_URI = http://192.168.3.2:11311
```
4. cd catkin_ws/
5. source devel/setup.zsh
6. Run ```roscore```
----
Test to make sure the connection is made by placing an object in the cage and running: ```rostopic echo /vicon/OBJ_NAME/OBJ_NAME``` on the ROS Desktop. The output should include the x-y-z position of the object in the cage. Remember that the drone you plan on controlling must have a corresponding VICON object that is created and oriented properly. You can find the the list of possible topics available using ```rostopic list```


## Keyboard Demo
Now that we have succesfully set up the VICON system it is time to get started on the first demo - controlling the position of our DJI Flamewheel using a keyboard. First, cd into your catkin workspace (this should be where your Freyja packages are located) and clone the following repositories:
```
cd catkin_ws/src
git clone git@github.com:lrse/ros-keyboard.git
git clone git@github.com:less-lab-uva/Drone-Demo.git
cd ..
source devel/setup.zsh
```
The ros-keyboard package is used for recording and interpreting keyboard actions

Once you have successfully cloned both repositories, you should make sure your catkin workspace can build. You can do this with:
```
catkin build
```
If you receive an error, you should run ```catkin clean -y``` and try building again. You should also make sure that the drone's Radio Telemetry Ground Module is plugged into your desktop.
After a successful clean, you are ready to fly your drone with your keyboard!
```
roslaunch Drone-Demo keyboard_demo.launch
```
If there are no errors, your terminal should output information such as the virtual cage that has been setup for the drone. Once the drone has been switched into computer control mode on the controller, you can now control the drone with your keyboard! *You'll notice that after you launch the program, a small blank GUI appears. In order for your keyboard actions to register, you must click on the blank GUI*

The arrow and WASD keys will move the drone 0.25 meters in each direction (North, East, South, and West). The '1' and '2' buttons will move the drone down and up by 0.25 meters. You can run ```rostopic echo /reference_state``` to see what location you are publishing to the drone. Remember that there is a virtual cage of 1.5 meters in each direction where the drone will not fly out of.

The launch file includes a node that publishes the positions of the drone to a topic called ```/visualization/drone```. While your ROS program is running, open a new terminal, source your catkin workspace, and then run ```rviz```. Once in rviz, you'll see an option to add a topic to the graph. Add ```/visualization/drone``` and you'll be able to see your drone fly around on your screen as well!

You can edit the virtual cage parameters in keyboard_demo.launch and how much the drone moves per click in keyboard_manager.py. If you decide to change the drone that is used from JOZI, make sure to change that in the parameters of the launch file and the corresponding files.


## TurtleBot3 Demo
Running the TurtleBot3 demo is very similar to the keyboard demo. Make sure you are in your catkin workspace, have sourced setup.zsh, and have a freshly built catkin workspace. Before launching the demo, make sure that your TurtleBot3 is within the cage and its position appears when running ```rostopic echo /vicon/BUGS/BUGS```. If you decide to change the TurtleBot that is used from BUGS, make sure to change that in the parameters of the launch file and the corresponding files.

```
roslaunch Drone-Demo turtle_demo.launch
```
If there are no errors, your terminal should output information such as the virtual cage that has been setup for the drone. Once the drone has been switched into computer control mode on the controller, the drone will hover above the TurtleBot as it travels through the cage! The 1.5m by 1.5m by 1.5m virtual cage still exists from the previous demo so even if the TurtleBot drives outside of the virtual cage, the drone will hover at the edge of the cage until the TurtleBot is back within the virtual cage. *You'll notice that after you launch the program, there is a 15 second pause between when the drone takes off and goes to hover over the TurtleBot. This is ensure the drone does not unpredictably crash into the TurtleBot*

You can also view the TurtleBot's position within rviz by viewing its position on the topic ```/visualization/turtle```
