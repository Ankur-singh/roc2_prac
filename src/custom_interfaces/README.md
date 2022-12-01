# Custom Interfaces

So far, we have being using built-in interfaces. But for large and complex projects these interfaces are not sufficient. Hence ROS2 provides us with tools to build our own interfaces. Here, I create one `msg` interface and two `srv` interface. To use them, we will have build them first.

```bash
colcon build --packages-select custom_interfaces
source ~/.bashrc
```

Once you have build the interfaces, you should be able to access them from ROS2 CLI as follows:

Lets start by looking at the message `Stock`
```bash
ros2 interface show custom_interfaces/msg/Stock
```

Expected output should look like this:
```
string name
int64 quantity
float32 price
```

Next we will look at srv `SetPrice.srv`
```bash
ros2 interface show custom_interfaces/srv/SetPrice
```

Expected Output:
```bash
float32 price
---
bool success
string message
```

Similarly, we can check `AddTwoStocks.srv`
```bash
ros2 interface show custom_interfaces/srv/AddTwoStocks
```

Expected Output:
```bash
Stock a
        string name
        int64 quantity
        float32 price
Stock b
        string name
        int64 quantity
        float32 price
---
Stock total
        string name
        int64 quantity
        float32 price
```

We can use these interfaces in our topics and services just like built-in interfaces.

The main goal here was to learn how to develop custom interfaces (both `msg` and `srv`) and use them. The best way to learn more about interfaces is to read *msg* and *srv* definitions from [ros2/example_interfaces](https://github.com/ros2/example_interfaces) and [ros2/common_interfaces](https://github.com/ros2/common_interfaces). 

Another good resource is [ROS interfaces Official docs](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html). Here you will find the list of built-in primitive data types that are currently supported. And if you are interested in building your own customer interfaces, then follow these tutorials: [Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) and [Implementing custom interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)