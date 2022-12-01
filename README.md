# ROS2 

## Setup

1. First, clone the repo

```bash
git clone https://github.com/Ankur-singh/ros2_prac
cd ros2_prac
```

2. Next, you will have to setup the development environment. ROS2 can be easily installed as a Debian Package by follow the instructions listed in the [official docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

or if you plan to use docker then here are all the steps

```bash
sudo docker run -it -v $(pwd):/home/ --name ros2_prac osrf/ros:humble-desktop
```

This should take you inside the docker. Once inside the docker, run the following commands

```bash
sudo apt-get update
sudo apt install python3-pip
pip install setuptools==58.2.0
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

You are all set to build and test your packages. For more detailed steps, refer [Setup Guide](https://github.com/Ankur-singh/myyolov7/wiki/ROS2-101) from here.

## Lessons:

I have create different packages

- [Hello Package](/src/hello_bot/)
- [Publisher Subscriber Package](/src/pub_sub_bot/)
- [Client Server Package](/src/client_server_bot/)
- [Custom Interfaces Package](/src/custom_interfaces/)
- Launch files

Here are so How-to guides:

- ROS2 [setup guide](/guides/setup.md)
- ROS2 [communication guide](/guides/communication.md)
- ROS2 [CLI guide](/guides/cli.md)