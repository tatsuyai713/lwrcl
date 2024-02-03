# RCL like Wrapper for Fast DDS

This software provides the wrapper API like rclcpp for Fast DDS.

## API List

### create_node

Create a Fast DDS node

```
intptr_t create_node(uint16_t domain_id);
```

### destroy_node

Destroy a Fast DDS node

```
void destroy_node(intptr_t node_ptr);
```

### spin

Infinite loop function. During infinite loop, execute callback functions.

```
void spin(intptr_t node_ptr);
```

### spin_once

Execute only one callback function.

```
void spin_once(intptr_t node_ptr);
```

### spin_some

Execute some queued callback function.

```
void spin_some(intptr_t node_ptr);
```

### stop_spin

Stop the infinite loop of spin function.

```
void stop_spin(intptr_t node_ptr);
```

### create_publisher

Create publisher.

```
intptr_t create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos);
```

### publish

Publish the topic of publisher.

```
void publish(intptr_t publisher_ptr, void *message);
```

### get_subscriber_count

Get subscriber number of the publisher.

```
int32_t get_subscriber_count(intptr_t publisher_ptr);
```

### destroy_publisher

Derstroy publisher.

```
void destroy_publisher(intptr_t publisher_ptr);
```

### create_subscription

Create subscription of the subscriber.

```
intptr_t create_subscription(intptr_t node_ptr, std::string message_type_name, std::string topic, eprosima::fastdds::dds::TopicQos &qos, std::function<void(void *)> callback);
```

### get_publisher_count

Get publisher number of subscription.

```
int32_t get_publisher_count(intptr_t subscriber_ptr);
```

### destroy_subscription 

Destroy subscription.

```
void destroy_subscription(intptr_t subscriber_ptr);
```

### create_timer

Create timer for cyclic calling the function.


```
intptr_t create_timer(intptr_t node_ptr, std::chrono::milliseconds period, std::function<void()> callback);
```

### destroy_timer

Destroy timer.

```
void destroy_timer(intptr_t timer_ptr);
```

### rcl_like_wrapper_init

RCL like wrapper initialization and registering a message type.

```
void rcl_like_wrapper_init(const MessageTypes &types);
```