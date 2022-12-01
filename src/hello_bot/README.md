## Hello Package

In this package (i.e. `hello_bot`), I created two nodes: `hello_node` and `counter_node`.

To use this package, we will have to build it. Run the following command

```bash
colcon build --packages-select hello_bot
source ~/.bashrc
```

**Note:** It is assumed that you have ROS2 installed in your system, and everything setup as instructed [here](https://github.com/Ankur-singh/myyolov7/wiki/ROS2-101)

Once you have build the package, you can run the nodes inside it. Lets discuss them one by one.

The `hello_node` simply prints `Hello from ROS2!` when you run it. Go ahead and give it a try:

```bash
ros2 run hello_bot hello_node
```

The `counter_node` does an extra thing. It maintains a counter variable, and keeps increasing its value every second. You will see a log message everything the counter variable is incremented.

```bash
ros2 run hello_bot counter_node
```

The important lesson here was to understand the end2end flow, starting from developing the node to building and using it.