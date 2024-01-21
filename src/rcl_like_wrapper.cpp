#include "node.hpp"
#include "qos.hpp"
#include "rcl_like_wrapper.hpp"

namespace rcl_like_wrapper {

namespace {
    MessageTypes message_types;
}

// Node
intptr_t node_create_node(uint16_t domain_id) {
  return reinterpret_cast<intptr_t>(new Node(domain_id));
}

void node_destroy_node(intptr_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->destroy();
}

void node_spin(intptr_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->spin();
}

void node_spin_once(intptr_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->spin_once();
}

void node_stop_spin(intptr_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->stop_spin();
}

// Create a publisher with the specified message type, topic, and QoS.
intptr_t publisher_create_publisher(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos& qos) {
  auto node = reinterpret_cast<Node *>(node_ptr);

  // Check if the message type exists in the map
  if (message_types.find(message_type_name) == message_types.end()) {
    // Handle error: Message type not found
    return 0;
  }

  // Creating a publisher with the specified message type, topic, and QoS.
  auto publisher = node->create_publisher(message_types.at(message_type_name), std::string("rt/") + topic, qos);

  // Check if the publisher creation was successful
  if (!publisher) {
    // Handle error: Publisher creation failed
    return 0;
  }

  return reinterpret_cast<intptr_t>(publisher);
}

void publisher_publish(intptr_t publisher_ptr, void* message) {
  auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
  // Publishing the message using the specified publisher.
  publisher->publish(message);
}

int32_t publisher_get_subscriber_count(intptr_t publisher_ptr) {
  auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
  // Getting the subscription count associated with the publisher.
  return publisher->get_subscriber_count();
}

void publisher_destroy_publisher(intptr_t publisher_ptr) {
    auto publisher = reinterpret_cast<Publisher*>(publisher_ptr);
    if (publisher) {
        delete publisher;
    }
}

// Create a Subscriber with the specified message type, topic, QoS, and callback function.
intptr_t subscriber_create_subscriber(intptr_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos& qos, std::function<void(void*)> callback) {
    auto node = reinterpret_cast<Node*>(node_ptr);

    // Check if the message type exists in the map
    if (message_types.find(message_type_name) == message_types.end()) {
        // Handle error: Message type not found
        return 0;
    }

    MessageType& message_type = message_types.at(message_type_name);

    // Creating a Subscriber with the specified message type, topic, QoS, and callback function.
    auto subscriber = node->create_subscriber(message_type, std::string("rt/") + topic, qos, [callback](void* message_data) {
        // Call the provided callback with the message data.
        callback(message_data);
    });

    // Check if the Subscriber creation was successful
    if (!subscriber) {
        // Handle error: Subscriber creation failed
        return 0;
    }

    return reinterpret_cast<intptr_t>(subscriber);
}

int32_t subscriber_get_publisher_count(intptr_t subscriber_ptr) {
  auto subscriber = reinterpret_cast<Subscriber *>(subscriber_ptr);
  // Getting the Publisher count associated with the Subscriber.
  return subscriber->get_publisher_count();
}

void subscriber_destroy_subscriber(intptr_t subscriber_ptr) {
    auto subscriber = reinterpret_cast<Subscriber*>(subscriber_ptr);
    if (subscriber) {
        delete subscriber;
    }
}

// Create a timer 
intptr_t timer_create_timer(intptr_t node_ptr, std::chrono::milliseconds period, std::function<void()> callback) {
  auto node = reinterpret_cast<Node *>(node_ptr);

    // Creating a Timer with callback function.
    auto timer = node->create_timer(period, callback);

    // Check if the Timer creation was successful
    if (!timer) {
        // Handle error: Timer creation failed
        return 0;
    }

    return reinterpret_cast<intptr_t>(timer);
}

void rcl_like_wrapper_init(const MessageTypes &types) {
  // Initializing MessageTypes.
  message_types = types;
}

}  // namespace rcl_like_wrapper
