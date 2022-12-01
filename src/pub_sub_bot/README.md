## Publisher Subscriber Package

Just like `hello_bot` package, we will start by building this package

```bash
colcon build --packages-select pub_sub_bot
source ~/.bashrc
```

This package has two nodes: `publisher_node` and `subscriber_node`

The `publisher_node` publishes updated in `bot_updates` topic every second. And the `subscriber_node` subscribes to `bot_updates` topic to receive these updates. Lets see them in action

In first terminal, run the publisher node
```bash
ros2 run pub_sub_bot publisher_node
```

and in the second terminal, run the subscriber node
```bash
ros2 run pub_sub_bot subscriber_node
```

You should see something similar to the talker-listener example in the official docs. 

The main aim here was to build a simple Publisher and Subscriber node to better understand topics and how to publish & subscribe to them. One good place to learn how to build Publisher and Subscriber nodes is [this tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).