#include "rclmodoki.hpp"

#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"

void myCallbackFunction(void* message) {
    sensor_msgs::msg::Image* my_message = static_cast<sensor_msgs::msg::Image*>(message);
    // Handle the received message
    std::cout << "Received data: " << my_message->header().stamp().sec() << std::endl;
}

int main() {
    using namespace rclmodoki;

    MessageType myMessageType(new sensor_msgs::msg::ImagePubSubType());
    MessageTypes messageTypes;

    messageTypes["sensor_msgs::msg::Image"] = myMessageType;
    rclmodoki_init(messageTypes);

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
        int data_value = 0;
        while (true) {
            // Simulate sending data periodically
            sensor_msgs::msg::Image* my_message = new sensor_msgs::msg::Image();
            my_message->header().stamp().sec() = data_value;
            data_value++;

            // Publish the message
            publisher_publish_impl(publisher_ptr, my_message);

            // Delete the message to avoid memory leak
            delete my_message;

            // Sleep for a while
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
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
