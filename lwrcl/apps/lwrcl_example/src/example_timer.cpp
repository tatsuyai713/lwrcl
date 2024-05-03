#include "lwrcl.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"

using namespace lwrcl;

#ifndef SENSOR_MSGS_MSG_IMAGETYPE_HPP
#define SENSOR_MSGS_MSG_IMAGETYPE_HPP
FAST_DDS_DATA_TYPE(sensor_msgs, msg, Image)
#endif // SENSOR_MSGS_MSG_IMAGETYPE_HPP

SIGNAL_HANDLER_DEFINE()

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

void myTimerFunction(sensor_msgs::msg::Image *my_message, Publisher<sensor_msgs::msg::Image>* publisher_ptr)
{
    static int data_value = 0;
    if (publisher_ptr == nullptr)
    {
        std::cerr << "Error: Invalid publisher pointer." << std::endl;
        return;
    }

    // Simulate sending data periodically
    my_message->header().stamp().sec() = data_value;
    my_message->header().stamp().nanosec() = data_value;
    my_message->header().frame_id() = "TEST";
    my_message->height() = 100;
    my_message->width() = 200;
    my_message->encoding() = "H263";
    my_message->is_bigendian() = false;
    my_message->step() = 1;
    my_message->data() = {0, 0, 0, 0, 0, 0};


    // Publish the message
    publisher_ptr->publish(my_message);
    data_value++;
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
    auto publisher_ptr = node->create_publisher<sensor_msgs::msg::Image>(&pub_message_type, "TESTTopic2", pub_topic_qos);
    if (publisher_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        return 1;
    }

    // Create a subscription with default QoS settings
    lwrcl::dds::TopicQos sub_topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
    auto subscriber_ptr = node->create_subscription<sensor_msgs::msg::Image>(&sub_message_type, "TESTTopic1", sub_topic_qos, myCallbackFunction);
    if (subscriber_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        return 1;
    }

    sensor_msgs::msg::Image pub_message;
    auto timer_ptr = node->create_timer(std::chrono::milliseconds(100), [&pub_message, publisher_ptr]()
                                            { myTimerFunction(&pub_message, publisher_ptr); });

    if (timer_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        return 1;
    }

    // Spin the node to handle incoming messages
    node->spin();

    return 0;
}
