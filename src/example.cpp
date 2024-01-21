#include "rcl_like_wrapper.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"

using namespace rcl_like_wrapper;

void myCallbackFunction(void *message)
{
    if (message == nullptr)
    {
        std::cerr << "Error: Received null message in callback." << std::endl;
        return;
    }

    sensor_msgs::msg::Image *my_message = static_cast<sensor_msgs::msg::Image *>(message);
    if (my_message == nullptr)
    {
        std::cerr << "Error: Failed to cast message to sensor_msgs::msg::Image." << std::endl;
        return;
    }

    // Handle the received message
    std::cout << "Received data: " << my_message->header().frame_id() << std::endl;
}

void myTimerFunction(void* ptr) {
    int test = 7;
    if (ptr == nullptr) {
        std::cerr << "Error: nullptr in callback." << std::endl;
        return;
    }
    void* publisher_ptr = ptr;

    int data_value = 0;
    // Simulate sending data periodically
    auto my_message = std::make_unique<sensor_msgs::msg::Image>();
    my_message->header().stamp().sec() = data_value;
    data_value++;

    // Publish the message
    if (publisher_ptr == nullptr) {
        std::cerr << "Error: Invalid publisher pointer." << std::endl;
        return;
    }
    printf("%d\n",test);
    publisher_publish(reinterpret_cast<intptr_t>(publisher_ptr), my_message.release()); // Release ownership and pass the raw pointer
}

// void publishMessage(intptr_t publisher_ptr) {
//     int data_value = 0;
//     auto start_time = std::chrono::steady_clock::now();

//     while (true) {
//         // Simulate sending data periodically
//         auto my_message = std::make_unique<sensor_msgs::msg::Image>();
//         my_message->header().stamp().sec() = data_value;
//         data_value++;

//         // Publish the message
//         if (publisher_ptr == 0) {
//             std::cerr << "Error: Invalid publisher pointer." << std::endl;
//             return;
//         }
//         publisher_publish(publisher_ptr, my_message.release());  // Release ownership and pass the raw pointer

//         // Calculate next publication time
//         start_time += std::chrono::milliseconds(100);

//         // Sleep until the next publication time
//         std::this_thread::sleep_until(start_time);
//     }
// }

int main()
{
    MessageTypes messageTypes;

    // Directly create a unique pointer
    std::unique_ptr<sensor_msgs::msg::ImagePubSubType> imagePubSubType = std::make_unique<sensor_msgs::msg::ImagePubSubType>();

    // Directly create rcl_like_wrapper::MessageType with a raw pointer
    messageTypes["sensor_msgs::msg::Image"] = rcl_like_wrapper::MessageType(imagePubSubType.get());

    rcl_like_wrapper_init(messageTypes);

    // Create a node with domain ID 0
    intptr_t node_ptr = node_create_node(0);
    if (node_ptr == 0)
    {
        std::cerr << "Error: Failed to create a node." << std::endl;
        return 1;
    }

    // Create a publisher with a topic named "MyTopic1" and default QoS
    dds::TopicQos topic_qos = dds::TOPIC_QOS_DEFAULT;
    intptr_t publisher_ptr = publisher_create_publisher(node_ptr, "sensor_msgs::msg::Image", "MyTopic1", topic_qos);
    if (publisher_ptr == 0)
    {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        node_destroy_node(node_ptr);
        return 1;
    }

    // Create a subscription with a topic named "MyTopic2" and default QoS
    intptr_t subscriber_ptr = subscriber_create_subscriber(
        node_ptr, "sensor_msgs::msg::Image", "MyTopic2", topic_qos, myCallbackFunction);
    if (subscriber_ptr == 0)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        publisher_destroy_publisher(publisher_ptr);
        node_destroy_node(node_ptr);
        return 1;
    }

    intptr_t timer_ptr = timer_create_timer(
        node_ptr, std::chrono::milliseconds(100), myTimerFunction, reinterpret_cast<void*>(publisher_ptr));
    if (timer_ptr == 0)
    {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        publisher_destroy_publisher(publisher_ptr);
        node_destroy_node(node_ptr);
        return 1;
    }

    // Start a thread to simulate a publisher sending data periodically
    // std::thread publisher_thread([&]() {
    //     publishMessage(publisher_ptr);
    // });

    // Main application loop
    while (true)
    {
        // Perform other tasks in your application

        // Check the number of publishers in the subscription
        int32_t publisher_count = subscriber_get_publisher_count(subscriber_ptr);
        std::cout << "Number of publishers: " << publisher_count << std::endl;

        // Spin the node to handle incoming messages
        node_spin_once(node_ptr);

        // Sleep for a while
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the publisher thread
    // publisher_thread.join();

    // Clean up
    subscriber_destroy_subscriber(subscriber_ptr);
    publisher_destroy_publisher(publisher_ptr);
    node_stop_spin(node_ptr);
    node_destroy_node(node_ptr);

    return 0;
}
