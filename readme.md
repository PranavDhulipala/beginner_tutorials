## ROS BEGINNER TUTORIALS

This is a repository based on ROS [tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) that covers how to write a publisher and subscriber node in C++.

A service was added to the talker node that changes the base output string.The talker node also includes the logging levels.

The nodes can now be called simultaneously by using the launch feature. The launch file accepts one command-line argument namely frequency which is of type integer that modifies the publisher's frequency.

## DEPENDENCIES

Ubuntu 16.04

ROS kinetic is required to run these nodes and can be installed by following the instructions provided [here](http://wiki.ros.org/kinetic/Installation)

## LICENSE

This program is under MIT License. A copy of the license can be obtained from [here](https://github.com/PranavDhulipala/beginner_tutorials/LICENSE) 

Copyright (c) 2017 Pranav Dhulipala
```bash
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
## BUILD INSTRUCTIONS

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone -b Week11_HW https://github.com/PranavDhulipala/beginner_tutorials
$ cd ..
$ catkin_make
```
## RUN INSTRUCTIONS

In a new terminal run
```
roscore
```
To run the talker, enter this command in a new terminal window.
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials talker frequency
```
frequency is an argument to the node which has an integer value, regulates the rate at which the output is dispayed on the screen. 

To run the listener, enter this command in a new terminal window.
```
cd catkin_ws
source devel/setup.bash
rosrun beginner_tutorials listener
```

## USING THE SERVICE

Make sure that both talker and lister nodes are running. In a new terminal enter the command ```rosservice list``` which will output the running services, you should be able to see the ``` /changeString``` 


```
cd catkin_ws
source devel/setup.bash
rosservice call /changeString name

```
## USING THE LAUNCH FILE

Run the below commands in the terminal
```
cd catkin_ws
source devel/setup.bash
roslaunch beginner_tutorials launch.launch frequency:=number
``` 
frequency is the argument that changes the publisher's frequency and number is of type integer which is passed as a value of the argument frequency.

## INSPECTING TF FRAMES

Make sure the roscore and talker are running,use the below commands to print the tf transforms on screen.

```
rosrun tf tf_echo /world /talk
```

To generate a PDF relative to the frames use the below command
```
rosrun tf view_frames
```
## TESTING 

From the workspace's parent folder enter one of the below commands in the terminal
```
catkin_make run_tests
```
or 

```
rostest beginner_tutorials talkerTest.launch
```
## USING ROSBAG

Run the command below to launch the nodes and record all the topics, exit manually.

```
roslaunch beginner_tutorials launch.launch record:=true
```
To play back the recording enter the following commands:

In a first terminal window:

```
rosrun beginner_tutorials listener
```

In a second terminal window:

```
rosbag play "rosbag file name"
```
The listener node now plays the previously recorded messages.
