# RCL-like Wrapper for Fast DDS

This library provides a simplified API similar to ROS 2's rclcpp for working with Fast DDS, enabling easier integration and management of nodes, publishers, subscribers, and timers within the Fast DDS ecosystem.

## Features

- Simplified node creation and management within a Fast DDS domain.
- Efficient message publishing and subscription handling with callback support.
- Seamless communication with ROS 2 topics without needing special modifications.
- Compatible with lightweight SBCs such as Raspberry Pi for easy integration with ROS 2 ecosystems.
- Periodic task execution using timers.
- Signal handling for graceful shutdown and resource cleanup.
- Supports custom message types for flexible communication needs.
- Executor for managing and spinning multiple nodes concurrently.

## API Overview

### Node Management

- **create_node**: Initializes a new Fast DDS node within the specified domain.
- **spin**: Continuously processes incoming messages and executes callbacks.
- **spin_once**: Processes a single message, if available.
- **spin_some**: Processes available messages without blocking.
- **stop_spin**: Stops the continuous message processing loop.

### Publisher

- **create_publisher**: Establishes a new message publisher on a specified topic.
- **publish**: Sends messages to the associated topic.
- **get_subscriber_count**: Retrieves the number of subscribers currently connected to the publisher.

### Subscriber

- **create_subscription**: Creates a subscription for receiving messages on a specified topic with a callback function.
- **get_publisher_count**: Counts the number of publishers to which the subscriber is connected.

### Timer

- **create_timer**: Sets up a timer to call a function at a specified interval.
- **stop_timer**: Halts the timer.

### Rate

- **Rate::sleep**: Delays execution to maintain a steady loop rate.

### Executor

- **Executor**: Manages and spins multiple nodes to facilitate concurrent message processing across different topics and services.
- **add_node**: Adds a node to the executor for management and spinning.
- **remove_node**: Removes a node from the executor.
- **spin**: Starts spinning all nodes managed by the executor, enabling concurrent message processing.
- **stop**: Stops spinning all nodes managed by the executor.

### Initialization

- **rcl_like_wrapper_init**: Initializes the wrapper with a set of predefined message types for communication.

## License

This project is a fork and has been modified under the terms of the Apache 2.0 license. The original work is also licensed under Apache 2.0. See the LICENSE file for more details.
