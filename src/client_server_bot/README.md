## Client Server Package

Lets start by building this package

```bash
colcon build --packages-select client_server_bot
source ~/.bashrc
```

Just like other packages, even this package has two nodes: `client_node` and  `server_node`.

The `server_node` takes two integers and returns the *sum* in response. Where as the `client_node` is used for sending requests to `server_node`. 

Lets start both the nodes,

In first terminal, run the server node
```bash
ros2 run pub_sub_bot server_node
```

Once the server is up and running, run the client node in the second terminal
```bash
ros2 run pub_sub_bot client_node
```

The goal here was to build a simple service and client node. For more in-depth understanding, follow [this](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html). Next step would be to learn how to develop more complex nodes with numerous clients and servers in the same node.