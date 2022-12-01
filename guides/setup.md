## Resources

- YT playlist : https://youtube.com/playlist?list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy
- Official Docs : https://docs.ros.org/en/humble/index.html

## Setup ROS2

Setting up ROS2 is quite a process in itself. There are several steps that you have to follow to get it up and running. 

1. Install ROS2
    
    ROS2 can be easily installed as a Debian Package by follow the instructions listed in the [official docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 

    Other option is to use docker. Just run the following command
    ```bash
    sudo docker run -it --name ros2_dev osrf/ros:humble-desktop
    sudo apt-get update # run from inside the docker
    ```
    
    One catch is that you will have to run `source /opt/ros/humble/setup.bash` every time you start a new terminal session. One work around is to add this line to `~/.bashrc`. So, that its executed automatically every time you start a new session. It can be easily done by running the following command 
    
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
    
    **Note:** I am assuming that you have added the `source /opt/ros/humble/setup.bash` to your `~/.bashrc`. If not, then you will have to run it in every terminal before you run any of the following command.
    
2. Try running some demo examples to make sure everything is working as expected. 
    
    Run the following in the first terminal
    
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
    
    Next, start another terminal and run the following to start the listener node
    
    ```bash
    ros2 run demo_nodes_py listener
    ```
    
    You will see that the everything that is published by talker is received by listener. This demo verifies that both C++ and Python APIs are working.
    
    If for some reason its not working, then try running [this](https://docs.ros.org/en/foxy/How-To-Guides/Installation-Troubleshooting.html#enable-multicast) to at least check if DDS communication is working. In the same link, you will find other problems and their solutions.
    

**Note:** you can also use docker instead of installing ROS2, here is the [link](https://hub.docker.com/r/osrf/ros) to the DockerHub profile 

Next, we will have to install the build tool to create our own ROS2 nodes

```bash
sudo apt-get install python3-colcon-common-extensions
```

To use auto-complete with colcon, we will have to add an extra line to our `~/.bashrc`

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

After this, you can either restart the terminal or just run `source ~/.bashrc` for making sure that these changes take place

## Setting up ROS2 workspaces

We will have to create a ROS workspace (generally named `ros2_ws`) and then create a source folder names `src` inside it. This `src` folder is were you will write all the code for your node. 

```bash
mkdir ros2_ws
cd ros2_ws
mkdir src
```

Next, we will run 

```bash
colcon build
```

This will try to build all the packages inside our `src` directory. But since we don’t have any package/code inside `src` directory, we will get `Summary: 0 packages finished` in the output. The main reason for running the build command is to check if everything is working as expected or not.

Note: If you get `EasyInstallDeprecationWarning` and/or `SetuptoolsDeprecationWarning` then you will have to downgrade setuptools to version `58.2.0`
```bash
sudo apt install python3-pip
pip install setuptools==58.2.0
```

After successful build, you should see 3 new directories in your workspace. The particular directory that is of interest to us is `install` directory. You will see a `setup.bash` file inside the `install` directory. We will have to source this file to use our custom ROS2 nodes. 

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

By this time, you should have 3 extra lines in your `.bashrc` file. This is how it should look

```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
```

Now, you are all set to start implementing your custom ROS2 node. 

## Creating ROS2 package - Python

Packages basically allows us to organize our code better. We can either create a Python package or C++ package. To create a package, run the following from inside your `src` directory:

```bash
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy
```

**Note:** No space character is allowed in package name.

`ament` is the build system. So, *colcon* our build tool will use *ament* (build system) to build our packages. Instead of `ament_python` one can also use `ament_cmake` if you want to develop a C++ package. `rclpy` is the ROS python package that will allow us to do ROS node development.

The above command will create a new directory with the package_name that you provided. You can `cd` into it to see its content. People with prior knowledge of python package will know that its a very basic template for python package with `setup.py`, `package.xml` and other files.

Now, you can go back to `src` directory and try running `colcon build` . This will try building your package. Make sure don’t get any error. Even if you get any error, you can easily resolve it with basic python knowledge.

Now, we can start developing our package.

## Implementation

We will be using `Node` class from `rclpy` python package to create nodes. The communication between the nodes is done used a simple PubSub mechanism. Nodes can publish data using topics and the other Nodes can simply subscribe to these topics to access the published data. 

For starters, we will make a very simple node that will just print “Hello World”. Put the following content inside `hello_node.py` file.

```python
import rclpy
from rclpy.node import Node

class HelloNode(Node):
	def __init__(self):
		super().__init__('hello_node')
		self.get_logger().info('Hello World')

def main():
	rclpy.init(args=None)
	node = HelloNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
```

Its just a python file, so we can try running it directly. It should work without any error. To run it using `ros2` cli, we will have to add this node to `setup.py` particularly in our `console_scripts` list. 

```python
entry_point = {
	"console_scripts" : [
		"hello_node = my_bot.hello_node:main",
	],
}
```

Every time you edit the `setup.py` you will have to run the *build* command and *source* the `~/.bashrc` file as follows:

```bash
colcon build --symlink-install
source ~/.bashrc
```

Now, you should be able to run our node using the ros2 CLi as follows:

```bash
ros2 run my_bot hello_node
```

Great, we got our very first ROS2 node up and running. 

Lets take and bit more ambitious challenge and try to make a simple *talker* node like the one we saw above (*demo_nodes_cpp*).

Create another python file called `counter_node.py` and paste the following content

```python
import rclpy
from rclpy.node import Node

class CounterNode(Node):
	def __init__(self):
		super().__init__('counter_node')
		self.counter = 0
		self.create_timer(1.0, self.callback)

	def callback(self):
		self.get_logger().info(f"Hello {self.counter}")
		self.counter += 1

def main():
	rclpy.init(args=None)
	node = CounterNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
```

And also update the `setup.py` file.

```python
entry_point = {
	"console_scripts" : [
		"hello_node = my_bot.hello_node:main",
		"counter_node = my_bot.counter_node.main",
	],
}
```

build again and source `~/.bashrc`

```bash
colcon build --symlink-install
source ~/.bashrc
```

Finally, its time to run the counter node

```bash
ros2 run my_bot counter_bot
```

The node is just printing the text on the screen. For this output to be accessible to other nodes as well, we will have to used *Topic.*

Here are all the important concepts that one needs to learn to better understand Node2Node communication in ROS:

- Topics
- Publisher and Subscriber
- RQT graph

The [official tutorials](https://docs.ros.org/en/humble/Tutorials.html) are a great please to learn about the above concepts.

### Object Detection Inference

For our use-case, I am referring the following repositories.  

- https://github.com/NVIDIA-AI-IOT/ros2_torch_trt
- https://github.com/NVIDIA-AI-IOT/yolov5_gpu_optimization

## Things I am yet to try out

- Communication between ROS2 nodes running in different machines or docker
  - https://nu-msr.github.io/navigation_site/lectures/remote_nodes.html
  - https://robotics.stackexchange.com/questions/11738/how-to-call-remote-ros-node-on-mobile-robot-through-laptop-using-wifi
  - https://husarion.com/tutorials/ros-tutorials/5-running-ros-on-multiple-machines/
- Making call to ROS2 nodes just like APIs (not necessarily other ROS2 node). Check [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite), [roslibpy](https://github.com/gramaziokohler/roslibpy), [ROSBridge](http://wiki.ros.org/rosbridge_suite), [rospy](http://wiki.ros.org/rospy) python client, [ROStful](https://github.com/pyros-dev/rostful), etc
- The complete inference architecture with realtime video streaming and TensorRT