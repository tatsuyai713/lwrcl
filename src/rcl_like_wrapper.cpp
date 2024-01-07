#include "node.hpp"
#include "qos.hpp"
#include "rcl_like_wrapper.hpp"

namespace rcl_like_wrapper {

static MessageTypes message_types;

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
  return reinterpret_cast<int64_t>(node->create_publisher(message_types.at(message_type_name), std::string("rt/") + topic, qos));
}

void publisher_publish_impl(int64_t publisher_ptr, void* message) {
  auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
  publisher->publish(message);
}

int32_t publisher_get_subscription_count(int64_t publisher_ptr) {
  auto publisher = reinterpret_cast<Publisher *>(publisher_ptr);
  return publisher->get_subscription_count();
}

// Subscription
int64_t subscription_create_subscription(int64_t node_ptr, std::string message_type_name, std::string topic, dds::TopicQos& qos, std::function<void(void*)> callback) {
    auto node = reinterpret_cast<Node*>(node_ptr);
    MessageType& message_type = message_types.at(message_type_name);

    return reinterpret_cast<int64_t>(node->create_subscription(message_type, std::string("rt/") + topic, qos, callback));
}

int32_t subscription_get_publisher_count(int64_t subscription_ptr) {
  auto subscription = reinterpret_cast<Subscription *>(subscription_ptr);
  return subscription->get_publisher_count();
}

void rcl_like_wrapper_init(const MessageTypes &types) {
  // Init MessageTypes
  message_types = types;
}

}
// namespace rcl_like_wrapper
