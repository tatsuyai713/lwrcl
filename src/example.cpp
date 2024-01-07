#include "rcl_like_wrapper.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"

using namespace rcl_like_wrapper;

void myCallbackFunction(void* message) {
    sensor_msgs::msg::Image* my_message = static_cast<sensor_msgs::msg::Image*>(message);
    // Handle the received message
    std::cout << "Received data: " << my_message->header().stamp().sec() << std::endl;
}

void publishMessage(int64_t publisher_ptr) {
    int data_value = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Simulate sending data periodically
        auto my_message = std::make_unique<sensor_msgs::msg::Image>();
        my_message->header().stamp().sec() = data_value;
        data_value++;

        // Publish the message
        publisher_publish_impl(publisher_ptr, my_message.release());  // Release ownership and pass the raw pointer

        // Calculate next publication time
        start_time += std::chrono::nanoseconds(100000000);

        // Sleep until the next publication time
        std::this_thread::sleep_until(start_time);

    }
}

int main() {
    MessageTypes messageTypes;

    // Directly create a unique pointer
    std::unique_ptr<sensor_msgs::msg::ImagePubSubType> imagePubSubType = std::make_unique<sensor_msgs::msg::ImagePubSubType>();

    // Directly create rcl_like_wrapper::MessageType with a raw pointer
    messageTypes["sensor_msgs::msg::Image"] = rcl_like_wrapper::MessageType(imagePubSubType.get());

    rcl_like_wrapper_init(messageTypes);

    // Create a node with domain ID 0
    int64_t node_ptr = node_create_node(0);

    // Create a publisher with a topic named "MyTopic1" and default QoS
    dds::TopicQos topic_qos = dds::TOPIC_QOS_DEFAULT;
    int64_t publisher_ptr = publisher_create_publisher(node_ptr, "sensor_msgs::msg::Image", "MyTopic1", topic_qos);

    // Create a subscription with a topic named "MyTopic2" and default QoS
    int64_t subscription_ptr = subscription_create_subscription(
        node_ptr, "sensor_msgs::msg::Image", "MyTopic2", topic_qos, myCallbackFunction
    );

    // Start a thread to simulate a publisher sending data periodically
    std::thread publisher_thread([&]() {
        publishMessage(publisher_ptr);
    });

    // Main application loop
    while (true) {
        // Perform other tasks in your application

        // Spin the node to handle incoming messages
        node_spin(node_ptr);

        // Check the number of publishers in the subscription
        int32_t publisher_count = subscription_get_publisher_count(subscription_ptr);
        std::cout << "Number of publishers: " << publisher_count << std::endl;

        // Sleep for a while
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the publisher thread
    publisher_thread.join();

    // Clean up
    node_stop_spin(node_ptr);
    node_destroy_node(node_ptr);

    return 0;
}
