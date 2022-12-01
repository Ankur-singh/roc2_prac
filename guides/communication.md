# ROS2 Node2Node communication

### 1. Nodes running on the same machine/container

ROS nodes that are running in the same system can directly communicate with each other. For example, run the talker in one terminal
```bash
ros2 run demo_nodes_cpp talker
```

and listener in another terminal
```bash
ros2 run demo_nodes_cpp listener
```

### 2. Nodes running in different container

ROS nodes in different machine/container can communicate with each other if both the machine/container are in the same network i.e. if you can ping the other machine and vice-a-versa. For example, you can create two containers as follows

In first terminal, 
```bash
sudo docker run -it --name ros1 osrf/ros:humble-desktop ros2 run demo_nodes_cpp talker
```

and in the second terminal,
```bash
sudo docker run -it --name ros2 osrf/ros:humble-desktop ros2 run demo_nodes_cpp listener
```

Both containers are able to communicate to each other because every time you create a new docker container, its added to default network driver (*bridge*). You can inspect the *bridge* driver by running `sudo docker network inspect bridge`. You should be able to see both `ros1` and `ros2`, and also their corresponding IP addresses.

If you don't want to use the `default` driver, then you can also specify your own driver by using the `--network` option in `docker run`. Both the containers should be added to the same network for them to be able to communicate without any issue. The above docker commands can be updated as follows:

In first terminal, 
```bash
sudo docker run -it --name ros1 --network rosnet osrf/ros:humble-desktop ros2 run demo_nodes_cpp talker
```

and in the second terminal,
```bash
sudo docker run -it --name ros2 --network rosnet osrf/ros:humble-desktop ros2 run demo_nodes_cpp listener
```

Both `ros1` and `ros2` are added to the `rosnet` network. **Note:** Don't use **host** as network name, because it's a reserved keyword.

Other and the simplest option is to create a docker-compose file. Docker compose automatically sets up a network. Each container in the compose file joins the default network and is both reachable by other containers on that network, and discoverable by them at a hostname identical to the container name. Create a `docker-compose.yml` file with the following content

```yml
version: '2'

services:
  talker:
    image: osrf/ros:humble-desktop
    command: ros2 run demo_nodes_cpp talker
  listener:
    image: osrf/ros:humble-desktop
    command: ros2 run demo_nodes_cpp listener
    depends_on:
      - talker
```

### 3. Nodes running in different machines (multi-machine)

But there are situations where ROS nodes are running in different machines. Computers running ROS nodes must be able to communicate with each other over a network if all computers are on a single local area network (LAN). The `rosmaster` process runs on one computer in the network and coordinates ROS nodes. Each Nodes contact the `rosmaster` to register their publishers and subscribers, and then directly connected to each other as needed.

One may ask, how does nodes know were `rosmaster` is running? Easy, we use `ROS_MASTER_URI` environment variable to tell all the nodes where to find the `rosmaster`. All nodes in the network, when launched, should have the same `ROS_MASTER_URI` in order to communicate on the same ROS network. `rosmaster` typically run on a development computer for visualization and diagnostics.

There is another important variable that we need to enable node to node communication. How do we know the IP address of the other node. We use another environment variable called `ROS_HOSTNAME` which contains the address of the node. When a node connects to `rosmaster` it provides the value of `ROS_HOSTNAME`. `rosmaster` uses this address to tell other nodes how to contact this node. If no `ROS_HOSTNAME` is provided, the hostname of the computer on which the node is running will be used.

**Note:** The `ROS_IP` variable is like `ROS_HOSTNAME` but contains the IP address of the node. Do not set this if using `ROS_HOSTNAME`

Resources:
- A more detailed explanation can be found [here](https://nu-msr.github.io/navigation_site/lectures/remote_nodes.html).
- Working examples can be found [here](https://robotics.stackexchange.com/a/12578), [here](https://husarion.com/tutorials/ros-tutorials/5-running-ros-on-multiple-machines/) and [here](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
- You can also find docker compose file for the same, [here](https://github.com/karadalex/ros-talkandlisten-py-docker/blob/master/docker-compose.yml) and [here](http://wiki.ros.org/docker/Tutorials/Compose)
