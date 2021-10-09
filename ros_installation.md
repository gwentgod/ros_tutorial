## ROS Installation

This instruction will help you install ROS with Docker on macOS.



### -1. Prerequisite

* macOS

  * You are ready

* Windows 10
  * **Home or Pro** 2004 (build 19041) or higher / **Enterprise or Education** 1909 (build 18363) or higher

    > To check your version, select **Win+ R**, type `winver`, select **OK**

  * BIOS-level hardware virtualization support enabled.

    > For more information, see [Virtualization](https://docs.docker.com/desktop/windows/troubleshoot/#virtualization-must-be-enabled).

  * WSL 2 feature enabled.

    > For more information, see [WSL 2](https://docs.microsoft.com/en-us/windows/wsl/install-win10).



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

* Windows

  ```powershell
  docker run -it --privileged \
  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros_shared:/root/shared \
  --name ros osrf/ros:melodic-desktop-full
  ```

* Mac - Intel chip

  ```zsh
  docker run -it --privileged -v ~/ros_shared:/root/shared --name ros osrf/ros:melodic-desktop-full
  ```
  
* Mac - Apple chip

  ```zsh
  docker run -it --privileged -v ~/ros_shared:/root/shared --name ros ros:melodic
  ```




### 3.  Setup

#### 3.1. Packages

* AMD64

  ```bash
  apt update
  apt upgrade
  ```
  
* Apple Chip

  ```bash
  apt update
  apt upgrade
  apt install ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-turtlesim
  ```

  



#### 3.2. Environment setup

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

There will be no output for the above two lines



#### 3.4 Testing

Try `roscore` in the container

You should see the following output:

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



* macOS

In Container:

```bash
echo "export DISPLAY=host.docker.internal:0" >> ~/.bashrc
source ~/.bashrc
```

There will be no output for the above two lines



On Host:

```zsh
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zprofile
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

* Windows

On Host:

Install [X server for Windows](https://sourceforge.net/projects/vcxsrv/). 

Install with default configs unless you know what you are doing.

**Restart your computer** after the installation is complted.



On Host:

```powershell
xhost +local:root
wsl2
export containerId=$(docker ps -l -q)
```

However, this compromises the access control to X server on your host. So with a little effort, someone could display something on your screen, capture user input, in addition to making it easier to exploit other vulnerabilities that might exist in X. If your computer contains sensitive or confidential files, please refer to other permission control methods provided on [this page](http://wiki.ros.org/docker/Tutorials/GUI).



In Container, try `rqt`

You should see a GUI window popup.
