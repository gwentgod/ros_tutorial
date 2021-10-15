## ROS_notes



* Node
* Docker & SSH 
* turtesim
* catkin workspace
  * package
* Hello world!
  * catkin_make
* Topic
  * CMakelist.txt & packages.xml
* \* Define messages
* Parameter
* \* Launch
* Service
* \* Action

---



### Node

* An executable file within a ROS package
* A process that carries out a specific task
* Nodes can be written in different languages, and run on different hosts distributedly
* Nodes must have unique names. If two nodes with the samename are launched, the previous one is kicked off.





### Run ROS 

* With Docker

  1. Start Docker Engine

  2. Start the container

     ```bash
     docker start ros
     ```

  3. Enter the ros container with

     ```bash
     docker exec -it ros bash
     ```

* SSH

  1. Connect to Wi-Fi: Xiaomi_CF12

  2. ```bash
     ssh icra@192.168.31.213
     ```





### catkin workspace

#### Init catkin_ws

```bash
mkdir -p /root/shared/catkin_ws/src
cd /root/shared/catkin_ws
catkin_make
echo "source /root/shared/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

```bash
catkin_create_pkg <name> [dependency0] [dependency1] ...
```



#### Tree

```
catkin_ws/
├── build
├── devel
└── src
    ├── CMakeLists.txt
    ├── pkg1
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── package.xml
    │   └── src
    │       ├── node1.cpp
    │       └── node2.py
    └── pkg2
        ├── CMakeLists.txt
        ├── action
        ├── config
        ├── include
        ├── msg
        ├── package.xml
        ├── src
        │   ├── node1.cpp
        │   └── node2.py
        └── srv
```



#### package

* Basic unit in ROS projects
  * All source code must be placed in package
* Packages must have unique names
* Including source code, configuration, dependencies, message definitions





### Hello World!

```c++
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "helloworld");
    ros::NodeHandle nh;
	  ros::Duration(1).sleep();

    ROS_INFO("Hello form ROS!");

    return 0;
}
```

```python
#!/usr/bin/env python3

import rospy as ros

ros.init_node('hello_py')
ros.sleep(1)

ros.loginfo("Hello from ROS!")
```



#### cmd

```bash
roscore
rosrun pkg exec
```





### Topic

* A "chatroom" provided by ROS Master that other nodes can write to and read from is called a topic
  * The node writing to the topic is called _Publisher_
  * The node reading from the topic is called _Subscriber_
  
* The structure of a message is `.msg`, and should be stored in `pkg/msg`

  ```
  ## e.g. geometry_msgs/Vector3
  
  float64 x
  float64 y
  float64 z
  
  
  ## eg. geometry_msgs/Twist
  
  geometry_msgs/Vector3 linear
  geometry_msgs/Vector3 angular
  ```
  
* Multiple nodes can publish to the same topic, and multiple nodes can also subscribe from the same topic

* The same node can be the publisher of multiple topics and the subscriber of multiple topics simultaneously

* It is possible for the message to be empty



#### cmd

```bash
rosmsg
rostopic
```



#### Coding

```c++
/* publisher */
ros::Publisher publisher = nh.advertise<pkg::Msg>("/topic", /*queue size=*/100);
pkg::Msg msg;
publisher.publish(msg);
```

```c++
/* subscriber */
void subCallback(const pkg::Msg::ConstPtr& msg);

ros::Subscriber subscriber = nh.subscribe("/topic", /*queue size=*/100, subCallback);

ros::spin(); // ros::spinOnce();
```



```python
## publisher
publisher = ros.Publisher("/topic", pkg.Msg, queue_size=100)
msg = pkg.Msg()
# edit msg
publisher.publish(msg)
```

```python
## subscriber
def subCallback(msg):
  pass

subscriber = ros.Subscriber("/topic", pkg.Msg, subCallback)
ros.spin()
```





### * Define messages

* Dependence: `message_generation`, `message_runtime`

  ```cmake
  # CMakeLists.txt
  find_package(catkin REQUIRED ... message_generation)
  add_action_files(FILES ... *.action)
  generate_messages(DEPENDENCIES ... message_runtime)
  ```

  ```xml
  <!-- package.xml -->
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```





### Parameter

* Parameter allows you to store and manipulate data on the ROS Parameter Server 

* The Parameter Server can store integers, floats, boolean, dictionaries, and lists

* You can save / load parameters to / from file. The file secture is `.yaml`

  ```
  ## e.g. icra2018.yaml
  
  image: icra2019.pgm
  resolution: 0.05
  origin: [0, 0, 0.000000]
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.196
  ```



#### cmd

```cmd
rosparam
```




#### Coding

```c++
Type val;
// getParam() returns a bool to check if retrieving the parameter succeeded
nh.getParam("/param", val);
nh.setParam("/param", val);
nh.deleteParam("/param");
```



```python
ros.set_param("/param", var)
if (ros.has_param("/param"):
	var = ros.get_param("/param")
rospy.delete_param("/param")

```





### * Launch

* Start multiple nodes at once
* Check if a `roscore` is running. If not, start a `roscore`
* Set parameters, including load parameters from files



#### cmd

```bash
roslaunch [pkg] [*.launch]
```



#### Coding

```xml
<launch>
    <param name="/param" value="value" type="int"/>
    <rosparam command="load" file="param.yaml"/>

	  <node pkg="package name" type="exec name" name="node name" args="-ab cd" output="screen"/>    
</launch>
```





### Service

* A "function" provided by a node that other nodes can call in ROS is called a service

  * The node that hosts the service is called _server_
  * The node that is calling the service is called _client_
  * The input of a service (function parameters) is called _request_
  * The output of a service (return values) is called _response_

* The structure of a request and response is `.srv`, and should be stored in `pkg/srv`

  ```
  ## e.g. turtlesim/Spawn
  
  // Request
  float32 x
  float32 y
  float32 theta
  string name
  ---
  // Response
  string name
  ```

* Multiple nodes can make requests (be the client) to the same service

* The same node can be the server of multiple services and the client of multiple (other) services simultaneously

* It is possible for the request and / or response to be empty

  Eg., if both are empty, then it is similar to a void function without any parameters

#### cmd

```bash
rossrv
rosservice
```



#### Coding

```c++
/* client */
ros::ServiceClient client = nh.serviceClient<pkg::Srv>("/service");
client.waitForExistence();

pkg::Srv srv;
// `call()` send request and wait for servic
// If the service call succeeded, call() will return true and the value in srv.response will be valid.
client.call(srv);
```

```c++
/* server */
bool Callback(pkg::srv::Request  &req, pkg::srv::Response &res);
	// return true after action succeed

ros::ServiceServer service = nh.advertiseService("/service", Callback);
ros::spin();
```



```python
## client
client = ros.ServiceProxy('/srv', pkg.Srv)
ros.wait_for_service('/srv')
request = pkg.SrvRequest()
# send request
# blocks until the servics finish, returns response
response = client(request)
```

```python
## server
def srvCallback(request):
  return response

server = ros.Service("/srv", pkg.Srv, srvCallback)
ros.spin()
```





### * Action

* Action is similar to Servics, but with extra ability to **cancel the request during execution** or **get periodic feedback** about how the request is progressing

  * The node that hosts the action is called _ActionServer_
  * The node that is calling the service is called _ActionClient_
  * The input of a action is called _goal_
  * The final output of a action is called _result_
  * The periodic feedback of a action is called _feedback_

* The structure of the goal, result and feedback is `.action`

  ```
  ## eg. CustomAction.action
  
  # Goal
  int32 target
  ---
  # Result
  bool succeeded
  ---
  # Feedback
  int32 progress
  ```

  Compilation outcome

  ```
  action file name: CustomAction.action
  
  type: CustomActionAction
  
  msgs:
  pkg/CustomActionAction
  pkg/CustomActionActionFeedback
  pkg/CustomActionActionGoal
  pkg/CustomActionActionResult
  pkg/CustomActionFeedback
  pkg/CustomActionGoal
  pkg/CustomActionResult
  ```

  

* Dependence: `actionlib`, `actionlib_msgs`

  ```cmake
  ## CMakeLists.txt
  find_package(catkin REQUIRED ... message_generation actionlib actionlib_msgs)
  add_action_files(FILES ... *.action)
  generate_messages(DEPENDENCIES ... actionlib_msgs)
  ```

  ```xml
  <!-- package.xml -->
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  ```



#### cmd

Actionlib does not provide command line tools. To send a goal to action server without action client, you can

* Use the rostopic pub direct in a terminal

  Action server will provide the following topics:

  ```
  /actServer/cancel
  /actServer/feedback
  /actServer/goal
  /actServer/result
  /actServer/status
  ```

  These are all normal topics and can be published / subscribed normally



* Use axclient from actionlib

  The actionlib offers a graphical way to send goal to action server. To use this interface, run in a terminal

  ```bash
  rosrun actionlib axclient.py /action_name
  ```



#### Coding

```c++
/* action server */
#include <actionlib/server/simple_action_server.h>

// actin file name: Act.action
pkg::ActFeedback feedback_msg;
pkg::ActResult result_msg;

void executeCallback(const pkg::ActGoalConstPtr& goal_msg)
{
	// check if action is died or preempted
	if (actServer.isPreemptRequested()) {
    // send result when preempted
    actServer.setPreempted(result_msg);
  }
	// send result when succeeded
  actServer.setSucceeded(result_msg);
	// send feedback
  actServer.publishFeedback(feedback_msg);
}

void preemptCallback();

actionlib::SimpleActionServer<pkg::ActAction> actServer(nh, "/act_name", executeCallback, /*auto start=*/false);
actServer.registerPreemptCallback(preemptCallback);
actServer.start();
```

``` c++
/* action client */
#include <actionlib/server/simple_action_client.h>

void startCallback();
void feedbackCallback(const pkg::ActFeedbackConstPtr& feedback);
void finishCallback(const actionlib::SimpleClientGoalState& state, 
                    const pkg::ActResultConstPtr& result);

// set the 2nd param true causes the client to spin its own thread
actionlib::SimpleActionClient<pkg::ActAction> actClient("/act_name", true);
pkg::ActGoal goal_msg;
actClient.sendGoal(goal_msg, &finishCallback, &startCallback, &feedbackCallback);
// blocks until this goal finishes or timeout. Returs finished ? true : false
actClient.waitForResult(ros::Duration(waiting_time));
// cancle the goal
actClient.cancelGoal();

```



```python
## action client
import actionlib

def feedbackCallback(feedback):
	pass
def activeCallback():
	pass
def doneCallback(status, result):
	pass

actClient = actionlib.SimpleActionClient('/act_name', ActAction)
actClient.wait_for_server()
goal = ActGoal()
actClient.send_goal(goal, feedback_cb=feedbackCallback,
                    active_cb=activeCallback, done_cb=doneCallback)
# blocks until this goal finishes or timeout. Returs finished ? true : false
actClient.wait_for_result(ros.Duration(wait_time))
actClient.cancel_goal()
```

```python
## action server
feedback = ActFeedback()
result = ActResult()

def executeCallback(goal):
  # check preemption
  if (actSrv.is_preempt_requested()):
		# send result when preempted
    actSrv.set_preempted(result)
  # send feedback
  actSrv.publish_feedback(feedback)
  # send result when succeeded
  actSrv.set_succeeded(result)
```





### And More

* [ROS Wiki](http://wiki.ros.org)
* [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem) & [Using rosed to edit files in ROS](http://wiki.ros.org/ROS/Tutorials/UsingRosEd)
* [Recording and playing back data](http://wiki.ros.org/ROS/Tutorials/Recording and playing back data) & [Reading messages from a bag file](http://wiki.ros.org/ROS/Tutorials/reading msgs from a bag file)
* [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting started with roswtf) - ROS debug tool
* [TF](http://wiki.ros.org/tf/Tutorials) - Tool for tracking coordinate frames
* [RoboRTS Tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/README) - The package we are working on



