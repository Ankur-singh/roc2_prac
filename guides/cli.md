# ROS2 CLI

ROS2 CLI is the best way to interact with different ROS components. The API is very similar to docker or K8s. Here are some commands that I found helpful for debugging and interacting with ROS components.

### Nodes

- `ros2 node list` : returns a list of all running nodes 
- `ros2 node info <node_name>` :  returns more information about the node like of subscribers, publishers, services, and actions (the ROS graph connections) that interact with that node. 

### Topics
- `ros2 topic list` : returns list of all topics that are currently active
- `ros2 topic list -t` : returns list of topics with the topic Interface
- `ros2 topic info <topic_name>` : returns topic type, number of publishers, and number of subscribers
- `ros2 topic hz <topic_name>` : returns rate a which data is published in the topic 
- `ros2 topic echo <topic_name>` : to see the data being published on the topic
- `ros2 topic pub <topic_name> <msg_type> '<args>'` : publish data onto a topic directly from the command line
    - `--once` option : publish one message then exit
    - `--rate <int>` option : publish the command in a steady stream at <int> Hz

### Interfaces
- `ros2 interface list` : returns list of all interfaces
- `ros2 interface show <interface_name>` : returns the schema details of the interface

### Services
- `ros2 service list` : returns list of all the services currently active in the system
- `ros2 service list -t` : returns all services with types
- `ros2 service type <service_name>` : returns types that describe how the request and response data of a service is structured
- `ros2 service call <service_name> <msg_type> '<args>'` : call service

### `rqt`

`rqt` is a GUI tool for ROS2. Everything done in `rqt` can be done on the command line, but it provides an easier, more user-friendly way to manipulate ROS2 elements. You can learn how to use it [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

- Source : https://roboticsbackend.com/ros2-service-cmd-line-tool-debug-ros2-services/


### Examples 

List all services
```bash
ros2 service list
```

Output:
```bash
/add_two_ints
/add_two_ints_server/describe_parameters
/add_two_ints_server/get_parameter_types
/add_two_ints_server/get_parameters
/add_two_ints_server/list_parameters
/add_two_ints_server/set_parameters
/add_two_ints_server/set_parameters_atomically
```


To know the *srv* interface details of `/add_two_ints` service
```bash
ros2 service type /add_two_ints
```

Output:
```bash
example_interfaces/srv/AddTwoInts
```

To learn about the interface schema
```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

Output:
```bash
int64 a
int64 b
---
int64 sum
```

Making call to `/add_two_ints` service
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 3}"
```

Output:
```bash
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=1, b=3)

response:
example_interfaces.srv.AddTwoInts_Response(sum=4)
```