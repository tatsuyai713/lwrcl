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

int data_value = 0;
void myTimerFunction(int test, void *ptr)
{

    if (ptr == nullptr)
    {
        std::cerr << "Error: nullptr in callback." << std::endl;
        return;
    }
    void *publisher_ptr = ptr;

    // Simulate sending data periodically
    auto my_message = std::make_unique<sensor_msgs::msg::Image>();
    my_message->header().stamp().sec() = data_value;
    data_value++;

    // Publish the message
    if (publisher_ptr == nullptr)
    {
        std::cerr << "Error: Invalid publisher pointer." << std::endl;
        return;
    }
    printf("%d\n", test);
    publish(reinterpret_cast<intptr_t>(publisher_ptr), my_message.release()); // Release ownership and pass the raw pointer
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

    int test = 100;
    intptr_t timer_ptr = create_timer(node_ptr, std::chrono::milliseconds(test), [test, publisher_ptr]()
                                            { myTimerFunction(test, reinterpret_cast<void *>(publisher_ptr)); });

    if (timer_ptr == 0)
    {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        destroy_publisher(publisher_ptr);
        destroy_subscription(subscriber_ptr);
        destroy_node(node_ptr);
        return 1;
    }

    // Spin the node to handle incoming messages
    spin(node_ptr);

    // Clean up
    destroy_subscription(subscriber_ptr);
    destroy_publisher(publisher_ptr);
    stop_spin(node_ptr);
    destroy_node(node_ptr);

    return 0;
}
