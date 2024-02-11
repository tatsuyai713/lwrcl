#include "ROSTypeImagePubSubExecutor.hpp"
#include <iostream>
#include <chrono>

ROSTypeImagePubSubExecutor::ROSTypeImagePubSubExecutor(uint16_t domain_number)
    : RCLWNode(domain_number), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000) {

    // Initialization of Image publisher subtype
    std::unique_ptr<sensor_msgs::msg::ImagePubSubType> image_pubsubtype =std::make_unique<sensor_msgs::msg::ImagePubSubType>();
    message_types_["sensor_msgs::msg::Image"] = MessageType(image_pubsubtype.release());
    rcl_like_wrapper_init(message_types_);

    counter_ = 0;
}

ROSTypeImagePubSubExecutor::~ROSTypeImagePubSubExecutor() {
}

bool ROSTypeImagePubSubExecutor::init(const std::string& config_file_path) {
    // Load configuration from YAML file
    YAML::Node node = YAML::LoadFile(config_file_path);
    YAML::Node config = node["config"];

    for (const auto& topic_node : config["publish_topics"]) {
        publish_topic_name_ = topic_node["name"].as<std::string>("default_topic");
        interval_ms_ = topic_node["interval_ms"].as<uint16_t>(1000);
        std::cout << "Topic name: " << publish_topic_name_ << ", Interval: " << interval_ms_ << " ms" << std::endl;
    }

    for (const auto& topic_node : config["subscribe_topics"]) {
        subscribe_topic_name_ = topic_node["name"].as<std::string>("default_topic");
        std::cout << "Topic name: " << subscribe_topic_name_ << std::endl;
    }

    if (interval_ms_ <= 0) {
        std::cerr << "Interval Time Error!" << std::endl;
        return false;
    }

    dds::TopicQos topic_qos = dds::TOPIC_QOS_DEFAULT;
    publisher_ptr_ = create_publisher(get_node_pointer(), "sensor_msgs::msg::Image", publish_topic_name_, topic_qos);
    if (!publisher_ptr_) {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        return false;
    }
    
    // Create a subscription with a topic named subscribe_topic_name_ and default QoS
    subscriber_ptr_ = create_subscription(get_node_pointer(), "sensor_msgs::msg::Image", subscribe_topic_name_, topic_qos, std::bind(&ROSTypeImagePubSubExecutor::callbackSubscribe, this, std::placeholders::_1));
    if (subscriber_ptr_ == 0)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        return false;
    }

    int test = 100;

    // Setup timer for periodic callback
    timer_callback_ = [this,test]() { this->callbackPublish(test); };
    timer_ptr_ = create_timer(get_node_pointer(), std::chrono::milliseconds(interval_ms_), timer_callback_);
    if (!timer_ptr_) {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        return false;
    }

    return true;
}

void ROSTypeImagePubSubExecutor::callbackPublish(int test) {
    // Update and publish message
    std::unique_ptr<sensor_msgs::msg::Image> publish_msg = std::make_unique<sensor_msgs::msg::Image>();
    publish_msg->header().stamp().sec() = (test + counter_);
    counter_++;

    if (!publisher_ptr_) {
        std::cerr << "Error: Invalid publisher pointer." << std::endl;
        return;
    }

    publish(reinterpret_cast<intptr_t>(publisher_ptr_), publish_msg.release());
}

void ROSTypeImagePubSubExecutor::callbackSubscribe(void *message)
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
    std::cout << "Received data: " << subscribe_topic_name_ << std::endl;
}