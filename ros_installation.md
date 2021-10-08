## ROS Installation

This instruction will help you install ROS with Docker on macOS.



### -1. Prerequisite

* This instruction only applies to macOS, 
* For Windows users, please
  1. Enable [WSL 2](https://docs.microsoft.com/en-us/windows/wsl/install-win10)
  2. [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) in your WSL
  3. Install [X server for Windows](https://sourceforge.net/projects/vcxsrv/)



### 0. References:

If you're just getting started using Docker with ROS, you are encouraged to make use of the following resources:

- [What is Docker](https://www.docker.com/whatisdocker/) - A page that will give you excellent high level overview of Docker and its purpose. 
- [Documentation](https://docs.docker.com/) - Browse the online documentation and references to find information about Docker's various functions and tools. 
- [Using Docker with ROS](http://wiki.ros.org/docker) - Official ROS wiki for using Docker with ROS.

---



### 1. Install Docker Desktop

<https://www.docker.com/get-started>



### 2. Run a ROS Container

In Terminal

* Intel chip

  ```zsh
  docker run -it --privileged -v ~/ros_shared:/root/shared --name ros osrf/ros:melodic-desktop-full
  ```
  
* Apple chip

  ```zsh
  docker run -it --privileged -v ~/ros_shared:/root/shared --name ros ros:melodic
  ```




### 3.  Setup

#### 3.1. Packages

* Intel Chip

  ```bash
  apt update
  apt upgrade
  ```
  
* Apple Chip

  ```bash
  apt update
  apt upgrade
  apt install oros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-turtlesim
  ```

  



#### 3.2. Environment setup

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



#### 3.4 Testing

Try `roscore` in the container

```
root@cac940d2ee67:/# roscore
... logging to /root/.ros/log/81e96716-12d5-11ec-809d-0242ac110003/roslaunch-cac940d2ee67-37.log

(omitted)

setting /run_id to 81e96716-12d5-11ec-809d-0242ac110003
process[rosout-1]: started with pid [58]
started core service [/rosout]

```



#### Congratulations, you've successfully installed ROS

---



### 4. Enable GUI

Optional, ~~GUI is for losers.~~

Sometimes GUI tools can be really helpful, but when you are directly operating robots, GUI is difficult to access.

In Container:

```bash
echo "export DISPLAY=host.docker.internal:0" >> ~/.bashrc
source ~/.bashrc
```

On Host:

```zsh
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ～/.zprofile
eval "$(/opt/homebrew/bin/brew shellenv)"
brew install socat
brew install xquartz
```

**Restart your computer** after installations are complted.



On Host, **EVERY TIME** before starting GUI in container:

```bash
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
```

This will run in foreground, so leave running and open a second terminal window to continue.


In Container, try `rqt`

You should see a GUI window popup.

