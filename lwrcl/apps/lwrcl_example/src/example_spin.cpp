#include "lwrcl.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"

using namespace lwrcl;

#ifndef SENSOR_MSGS_MSG_IMAGETYPE_HPP
#define SENSOR_MSGS_MSG_IMAGETYPE_HPP
FAST_DDS_DATA_TYPE(sensor_msgs, msg, Image)
#endif // SENSOR_MSGS_MSG_IMAGETYPE_HPP

SIGNAL_HANDLER_DEFINE()

// Callback function for handling received messages
void myCallbackFunction(sensor_msgs::msg::Image *message)
{
    if (message == nullptr)
    {
        std::cerr << "Error: Received null message in callback." << std::endl;
        return;
    }
    // Print the received image data
    std::cout << "Received Image data: ";
    std::cout << "Width: " << message->width() << ", ";
    std::cout << "Height: " << message->height() << ", ";
    std::cout << "Encoding: " << message->encoding() << std::endl;
}

int main()
{
    SIGNAL_HANDLER_INIT()

    // MessageType
    sensor_msgs::msg::ImageType sub_message_type;
    sensor_msgs::msg::ImageType pub_message_type;

    // Create a node with domain ID 0
    std::shared_ptr<Node>node = std::make_shared<Node>(0);

    // Create a publisher with default QoS settings
    lwrcl::dds::TopicQos pub_topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
    auto publisher_ptr = node->create_publisher<sensor_msgs::msg::Image>(&pub_message_type, "TESTTopic1", pub_topic_qos);
    if (publisher_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        return 1;
    }

    // Create a subscription with default QoS settings
    lwrcl::dds::TopicQos sub_topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
    auto subscriber_ptr = node->create_subscription<sensor_msgs::msg::Image>(&sub_message_type, "TESTTopic2", sub_topic_qos, myCallbackFunction);
    if (subscriber_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        return 1;
    }

    int data_value = 0;
    Rate rate(Duration(std::chrono::milliseconds(100))); // Set rate to 100 milliseconds

    struct timespec curTime, lastTime;
    clock_gettime(CLOCK_REALTIME, &lastTime);
    
    // Main application loop
    while (ok())
    {
        // Check the number of publishers
        int32_t publisher_count = subscriber_ptr->get_publisher_count();
        std::cout << "Number of publishers: " << publisher_count << std::endl;

        clock_gettime(CLOCK_REALTIME, &curTime);
        // Print the interval between the last two messages
        if (curTime.tv_nsec < lastTime.tv_nsec)
        {
            printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec - 1, curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
        }
        else
        {
            printf("Interval = %10ld.%09ld\n", curTime.tv_sec - lastTime.tv_sec, curTime.tv_nsec - lastTime.tv_nsec);
        }
        lastTime = curTime;

        // Simulate sending data periodically
        sensor_msgs::msg::Image pub_message;
        pub_message.header().stamp().sec() = data_value;
        pub_message.header().stamp().nanosec() = data_value;
        pub_message.header().frame_id() = "TEST";
        pub_message.height() = 100;
        pub_message.width() = 200;
        pub_message.encoding() = "H263";
        pub_message.is_bigendian() = false;
        pub_message.step() = 1;
        pub_message.data() = {0, 0, 0, 0, 0, 0};

        // Publish the data
        publisher_ptr->publish(&pub_message);

        // Handle incoming messages
        node->spin_some();

        data_value++;
        rate.sleep();
    }

    return 0;
}
