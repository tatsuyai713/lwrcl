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
- Executors for managing and spinning multiple nodes concurrently.

## API Overview
  
### Node Management

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

## Time, Duration, Clock, and Rate Implementation

This section outlines the implementation details of the Time, Duration, Clock, and Rate classes, which are essential for handling timing and scheduling within the system.

### Time Implementation

The `Time` class provides functionality to represent a point in time or a duration of time with nanosecond precision.

- **Constructors:**
  - Default constructor initializes time to 0 nanoseconds.
  - Constructor accepting an `int64_t` for nanoseconds.
  - Constructor accepting an `int32_t` for seconds and a `uint32_t` for additional nanoseconds.

- **Methods:**
  - `nanoseconds()`: Returns the time as an integer number of nanoseconds.
  - `seconds()`: Returns the time as a floating-point number of seconds.

### Duration Implementation

The `Duration` class represents a time span with nanosecond precision.

- **Constructors:**
  - Default constructor initializes duration to 0 nanoseconds.
  - Constructor accepting an `int64_t` for nanoseconds.
  - Constructor accepting an `int32_t` for seconds and a `uint32_t` for additional nanoseconds.

- **Methods:**
  - `nanoseconds()`: Returns the duration as an integer number of nanoseconds.
  - `seconds()`: Returns the duration as a floating-point number of seconds.

### Clock Implementation

The `Clock` class is used to access the current time, based on the clock type.

- **Constructor:** Accepts a `ClockType` enumeration to specify the clock type (e.g., `SYSTEM_TIME`).
- **Methods:**
  - `now()`: Returns the current time as a `Time` object, based on the clock's type.
  - `get_clock_type()`: Returns the type of the clock.

### Rate Implementation

The `Rate` class is designed to maintain a specified rate of loop iteration, using sleep to delay execution.

- **Constructor:** Accepts a `Duration` object that specifies the desired period between iterations.
- **Method:**
  - `sleep()`: Sleeps until the next iteration should start, adjusting for any drift to maintain the rate.

These classes provide a foundational framework for precise time management and rate control in the system, allowing for time-based scheduling and execution control.

# Executors for Fast DDS

Executors play a crucial role in the RCL-like wrapper for Fast DDS, allowing for concurrent message processing and event handling across multiple nodes. Inspired by the ROS 2 executor concept, these executors facilitate the management and operation of nodes, enabling efficient communication within the Fast DDS ecosystem.

## SingleThreadedExecutor

The `SingleThreadedExecutor` manages and spins multiple nodes sequentially within a single thread. This executor is designed for simplicity and is best suited for scenarios where tasks need to be executed in a specific order without the overhead of multi-threading.

### Features

- **Sequential Processing:** Executes node callbacks one at a time, ensuring that message processing does not overlap.
- **Simplicity:** Easier to debug and maintain due to the single-threaded nature of operation.
- **Use Case:** Ideal for simpler systems where concurrent message processing is not critical, or for tasks that must be executed in order.

### Key Functions

- **add_node(Node* node_ptr):** Integrates a node into the executor's workflow.
- **remove_node(Node* node_ptr):** Detaches a node from the executor.
- **spin():** Begins the sequential processing of messages for all nodes managed by the executor.
- **stop_spin():** Halts the processing loop, ensuring all nodes are gracefully stopped.

## MultiThreadedExecutor

The `MultiThreadedExecutor` extends the functionality of the SingleThreadedExecutor by allowing nodes to be spun in parallel across multiple threads. This executor is capable of handling more complex systems where tasks need to run concurrently, optimizing performance and responsiveness.

### Features

- **Concurrent Processing:** Enables nodes to process messages and handle events simultaneously across different threads.
- **Scalability:** Efficiently manages a larger number of nodes, making it suitable for more complex applications.
- **Flexibility:** Offers the ability to handle variable loads and tasks that are independent of each other.
- **Use Case:** Best for applications requiring real-time processing or when multiple nodes need to operate independently without blocking each other.

### Key Functions

- **add_node(Node* node_ptr):** Adds a node to be managed concurrently by the executor.
- **remove_node(Node* node_ptr):** Removes a node from the concurrent processing pool.
- **spin():** Starts concurrent message processing for all nodes, leveraging multi-threading to achieve parallel execution.
- **stop_spin():** Stops all threads and ensures a clean shutdown of node operations.

## Choosing Between Executors

- **SingleThreadedExecutor** is recommended for simpler or linear workflows where task order is important and system resources are limited.
- **MultiThreadedExecutor** is ideal for complex, real-time systems requiring parallel data processing and where tasks can safely execute independently of one another.

By selecting the appropriate executor based on your application's requirements, you can optimize your Fast DDS application for performance, simplicity, or a balance of both.

## RCLWNode: Enhanced Node Management

`RCLWNode` provides an abstraction layer for creating and managing nodes within the Fast DDS ecosystem, mirroring the functionality found in ROS 2's `rclcpp::Node`. It simplifies the interaction with the underlying DDS layer, offering a user-friendly interface for developing distributed systems that communicate over DDS.

### Key Features

- **Simplified Node Creation**: Facilitates the setup of DDS nodes by abstracting away the complexity of DDS configurations.
- **Message Publishing and Subscription**: Offers easy-to-use methods for publishing messages to topics and subscribing to topics with callback functions for received messages.
- **Timer Management**: Allows the scheduling of periodic tasks, making it easier to handle time-driven operations.
- **Signal Handling**: Integrates signal handling for graceful shutdown and cleanup, enhancing the robustness of applications.
- **Executors Compatibility**: Designed to work seamlessly with the `Executors` for concurrent message processing across multiple nodes.

### Using RCLWNode with Executors

Integrating `RCLWNode` with Executors in the Fast DDS environment facilitates the effective management and operation of multiple nodes. This setup is essential for developing distributed applications that require efficient multitasking and the ability to handle messages from multiple sources in parallel. The use of Executors, specifically the `SingleThreadedExecutor` and `MultiThreadedExecutor`, plays a pivotal role in how messages are processed and how nodes communicate within a Fast DDS domain.

#### SingleThreadedExecutor vs. MultiThreadedExecutor

- **SingleThreadedExecutor**: This executor processes messages for all nodes sequentially in a single thread. It is simpler and easier to debug but might not be suitable for applications requiring high-throughput message processing or real-time responsiveness. It ensures that message callbacks for each node are executed in the order they are received, which can be critical for certain types of data processing where order matters.

- **MultiThreadedExecutor**: Designed for more complex scenarios, this executor allows multiple nodes to be spun concurrently across different threads. This is particularly useful in systems where nodes operate independently or when the application demands real-time processing. The MultiThreadedExecutor enhances throughput and responsiveness by leveraging parallel processing capabilities.

The choice between SingleThreadedExecutor and MultiThreadedExecutor depends on the specific requirements of your application, including the need for real-time data processing, the complexity of the tasks performed by each node, and the overall system architecture.

The `MultiThreadedExecutor` is especially suitable for applications that demand high performance and scalability, where tasks across different nodes do not need to be executed in a strict sequence. It exemplifies how Fast DDS can be employed to build robust and high-throughput distributed applications, making it an invaluable tool for developers working on advanced systems within the ROS 2 ecosystem and beyond.

## License

This project is a fork and has been modified under the terms of the Apache 2.0 license. The original work is also licensed under Apache 2.0. See the LICENSE file for more details.

