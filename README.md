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

## Executor: Concurrent Node Management

The `Executor` is a core component designed to facilitate concurrent message processing across multiple nodes within the Fast DDS domain. Inspired by ROS 2's executor concept, it allows users to manage and spin multiple nodes, thereby handling events from various sources like subscriptions, timers, and service clients concurrently. This feature is particularly useful in complex systems where different modules or functionalities need to operate in parallel without blocking each other.

### Key Functions

#### add_node(intptr_t node_ptr)

- **Purpose**: Adds a node to the executor's management list. Once added, the node is eligible for concurrent spinning alongside other nodes managed by the executor.
- **Parameters**:
  - node_ptr: Pointer to the node to be managed. This pointer is obtained through the create_node function.
- **Usage Scenario**: When you have initialized multiple nodes and wish to ensure they are concurrently processing messages or events.

#### remove_node(intptr_t node_ptr)

- **Purpose**: Removes a previously added node from the executor's management. This is useful for dynamically adjusting which nodes are active based on runtime conditions.
- **Parameters**:
  - node_ptr: Pointer to the node to be removed from the executor's management.
- **Usage Scenario**: In scenarios where nodes are dynamically added or removed based on application logic or system state.

#### spin()

- **Purpose**: Initiates the spinning of all nodes managed by the executor, enabling them to concurrently process messages, service requests, and timer callbacks. This function blocks and should ideally be run in its dedicated thread if the application needs to perform other tasks concurrently.
- **Usage Scenario**: To start the message processing loop for all managed nodes. Typically called after all desired nodes have been added to the executor.

#### stop()

- **Purpose**: Stops the spinning of all nodes managed by the executor. It ensures that all message processing loops are gracefully terminated.
- **Usage Scenario**: When the application is shutting down or when you need to temporarily halt message processing to perform maintenance or reconfiguration tasks.

## RCLWNode: Enhanced Node Management

`RCLWNode` provides an abstraction layer for creating and managing nodes within the Fast DDS ecosystem, mirroring the functionality found in ROS 2's `rclcpp::Node`. It simplifies the interaction with the underlying DDS layer, offering a user-friendly interface for developing distributed systems that communicate over DDS.

### Key Features

- **Simplified Node Creation**: Facilitates the setup of DDS nodes by abstracting away the complexity of DDS configurations.
- **Message Publishing and Subscription**: Offers easy-to-use methods for publishing messages to topics and subscribing to topics with callback functions for received messages.
- **Timer Management**: Allows the scheduling of periodic tasks, making it easier to handle time-driven operations.
- **Signal Handling**: Integrates signal handling for graceful shutdown and cleanup, enhancing the robustness of applications.
- **Executor Compatibility**: Designed to work seamlessly with the `Executor` for concurrent message processing across multiple nodes.

### Using RCLWNode with Executor

The integration of `RCLWNode` with the `Executor` enables efficient management and spinning of multiple nodes, allowing for concurrent operations and message handling in a Fast DDS environment. This setup is particularly beneficial for complex applications that require multitasking or handling messages from various sources simultaneously.

`RCLWNode`, in combination with the `Executor`, offers a powerful and intuitive framework for developing distributed applications using Fast DDS. It abstracts the complexities of direct DDS interactions, providing a ROS 2-like interface for easier and more efficient system development.

#### Example Usage

This example demonstrates how to create two `RCLWNode` instances and use an `Executor` to manage them concurrently.

```cpp
#include "rcl_like_wrapper.hpp"

int main() {
  // Initialize the wrapper and executor
  rcl_like_wrapper::Executor executor;

  // Create the first node with a specific domain ID
  auto node1 = std::make_shared<rcl_like_wrapper::RCLWNode>(0); // Domain ID 0
  auto node2 = std::make_shared<rcl_like_wrapper::RCLWNode>(0); // Same domain, different node

  // Add nodes to the executor for concurrent management
  executor.add_node(node1->get_node_pointer());
  executor.add_node(node2->get_node_pointer());

  // Start concurrent processing with the executor
  executor.spin();

  return 0;
}
```

In this example, `RCLWNode` instances are created for two different nodes within the same DDS domain. These nodes are then added to an `Executor`, which manages their lifecycle and ensures that they process messages concurrently. This approach simplifies the development of distributed systems with Fast DDS, providing a familiar interface for those accustomed to ROS 2 development patterns.


### Initialization

- **rcl_like_wrapper_init**: Initializes the wrapper with a set of predefined message types for communication.

## License

This project is a fork and has been modified under the terms of the Apache 2.0 license. The original work is also licensed under Apache 2.0. See the LICENSE file for more details.
