#include "node.hpp"
#include "qos.hpp"
#include "rcl_like_wrapper.hpp"

namespace rcl_like_wrapper {

namespace {
    MessageTypes message_types;
}

// Node
int64_t node_create_node(int64_t domain_id) {
  return reinterpret_cast<int64_t>(new Node(domain_id));
}

void node_destroy_node(int64_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->destroy();
}

void node_spin(int64_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->spin();
}

void node_spin_once(int64_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->spin_once();
}

void node_stop_spin(int64_t node_ptr) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  node->stop_spin();
}

// Publisher
int64_t publisher_create_publisher(int64_t node_ptr, std::string message_type_name, std::string topic,
                                 dds::TopicQos& qos) {
  auto node = reinterpret_cast<Node *>(node_ptr);
  // Creating a publisher with the specified message type, topic, and QoS.
  return reinterpret_cast<int64_t>(node->create_publisher(message_types.at(message_type_name), std::string("rt/") + topic, qos));
}

void publisher_publish_impl(int64_t publisher_ptr, void* message) {
  auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
  // Publishing the message using the specified publisher.
  publisher->publish(message);
}

int32_t publisher_get_subscription_count(int64_t publisher_ptr) {
  auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
  // Getting the subscription count associated with the publisher.
  return publisher->get_subscription_count();
}

// Subscription
int64_t subscription_create_subscription(int64_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos& qos, std::function<void(void*)> callback) {
    auto node = reinterpret_cast<Node*>(node_ptr);
    MessageType& message_type = message_types.at(message_type_name);

    // Creating a subscription with the specified message type, topic, QoS, and callback function.
    auto subscription = node->create_subscription(message_type, std::string("rt/") + topic, qos, [callback](void* message_data) {
        // Call the provided callback with the message data.
        callback(message_data);
    });

    return reinterpret_cast<int64_t>(subscription);
}

int32_t subscription_get_publisher_count(int64_t subscription_ptr) {
  auto subscription = reinterpret_cast<Subscription *>(subscription_ptr);
  // Getting the publisher count associated with the subscription.
  return subscription->get_publisher_count();
}

void rcl_like_wrapper_init(const MessageTypes &types) {
  // Initializing MessageTypes.
  message_types = types;
}

}
// namespace rcl_like_wrapper
