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

int main()
{
    MessageTypes messageTypes;

    // Directly create a unique pointer
    std::unique_ptr<sensor_msgs::msg::ImagePubSubType> imagePubSubType = std::make_unique<sensor_msgs::msg::ImagePubSubType>();

    // Directly create rcl_like_wrapper::MessageType with a raw pointer
    messageTypes["sensor_msgs::msg::Image"] = rcl_like_wrapper::MessageType(imagePubSubType.get());

    rcl_like_wrapper_init(messageTypes);

    // Create a node with domain ID 0
    intptr_t node_ptr = create_node(0);
    if (node_ptr == 0)
    {
        std::cerr << "Error: Failed to create a node." << std::endl;
        return 1;
    }

    // Create a publisher with a topic named "MyTopic1" and default QoS
    dds::TopicQos topic_qos = dds::TOPIC_QOS_DEFAULT;
    intptr_t publisher_ptr = create_publisher(node_ptr, "sensor_msgs::msg::Image", "MyTopic1", topic_qos);
    if (publisher_ptr == 0)
    {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        destroy_node(node_ptr);
        return 1;
    }

    // Create a subscription with a topic named "MyTopic2" and default QoS
    intptr_t subscriber_ptr = create_subscription(
        node_ptr, "sensor_msgs::msg::Image", "MyTopic2", topic_qos, myCallbackFunction);
    if (subscriber_ptr == 0)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        destroy_publisher(publisher_ptr);
        destroy_node(node_ptr);
        return 1;
    }

    int data_value = 0;
    Rate rate(std::chrono::milliseconds(100));

    struct timespec curTime, lastTime;
    clock_gettime(CLOCK_REALTIME, &lastTime);
    // Main application loop
    while (true)
    {
        // Perform other tasks in your application

        // Check the number of publishers in the subscription
        int32_t publisher_count = get_publisher_count(subscriber_ptr);
        std::cout << "Number of publishers: " << publisher_count << std::endl;

        // Simulate sending data periodically
        auto my_message = std::make_unique<sensor_msgs::msg::Image>();
        my_message->header().stamp().sec() = data_value;
        data_value++;

        clock_gettime(CLOCK_REALTIME, &curTime);
        if (curTime.tv_nsec < lastTime.tv_nsec)
        {
            printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec - 1, curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
        }
        else
        {
            printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec, curTime.tv_nsec - lastTime.tv_nsec);
        }
        lastTime = curTime;

        publish(publisher_ptr, my_message.release()); // Release ownership and pass the raw pointer

        // Spin the node to handle incoming messages
        spin_once(node_ptr);

        rate.sleep();
    }

    // Clean up
    destroy_subscription(subscriber_ptr);
    destroy_publisher(publisher_ptr);
    stop_spin(node_ptr);
    destroy_node(node_ptr);

    return 0;
}
