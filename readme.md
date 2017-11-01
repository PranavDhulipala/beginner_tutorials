## ROS BEGINNER TUTORIALS

This is a repository based on ROS [tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) that covers how to write a publisher and subscriber node in C++.

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
$ git clone --recursive https://github.com/PranavDhulipala/beginner_tutorials
$ cd ..
$ catkin_make
```
## RUN INSTRUCTIONS

```
roscore
```
To run the talker, enter this command in a new terminal window.
```
rosrun beginner_tutorials talker
```
To run the listener, enter this command in a new terminal window.
```
rosrun beginner_tutorials listener
```
