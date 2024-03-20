#include "rcl_like_wrapper.hpp"
#include "sensor_msgs/msg/Image.h"
#include "sensor_msgs/msg/ImagePubSubTypes.h"

using namespace rcl_like_wrapper;

FAST_DDS_CUSTOM_TYPE(sensor_msgs, msg, Image)

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
    // MessageType
    sensor_msgs::msg::ImageType sub_message_type;
    sensor_msgs::msg::ImageType pub_message_type;

    // Create a node with domain ID 0
    std::shared_ptr<Node>node_ptr = std::make_shared<Node>(0);

    // Create a publisher with default QoS settings
    rcl_like_wrapper::dds::TopicQos pub_topic_qos = rcl_like_wrapper::dds::TOPIC_QOS_DEFAULT;
    auto publisher_ptr = node_ptr->create_publisher<sensor_msgs::msg::Image>(&pub_message_type, "TESTTopic1", pub_topic_qos);
    if (publisher_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        return 1;
    }

    // Create a subscription with default QoS settings
    rcl_like_wrapper::dds::TopicQos sub_topic_qos = rcl_like_wrapper::dds::TOPIC_QOS_DEFAULT;
    auto subscriber_ptr = node_ptr->create_subscription<sensor_msgs::msg::Image>(&sub_message_type, "TESTTopic2", sub_topic_qos, myCallbackFunction);
    if (subscriber_ptr == nullptr)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        return 1;
    }

    int data_value = 0;
    Rate rate(Duration(100000000)); // Set rate to 100 milliseconds

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
        node_ptr->spin_once();

        data_value++;
        rate.sleep();
    }

    return 0;
}
