#include "ROSTypeImagePubSubMono.hpp"
#include <iostream>
#include <chrono>

ROSTypeImagePubSubMono::ROSTypeImagePubSubMono(uint16_t domain_number)
    : RCLWNode(domain_number), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000) {

    // Initialization of Image publisher subtype
    std::unique_ptr<sensor_msgs::msg::ImagePubSubType> image_pubsubtype =std::make_unique<sensor_msgs::msg::ImagePubSubType>();
    message_types_["sensor_msgs::msg::Image"] = MessageType(image_pubsubtype.release());
    rcl_like_wrapper_init(message_types_);

    counter_ = 0;
    
    gray_msg_ = std::make_shared<sensor_msgs::msg::Image>();
}

ROSTypeImagePubSubMono::~ROSTypeImagePubSubMono() {
}

bool ROSTypeImagePubSubMono::init(const std::string& config_file_path) {
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
    subscriber_ptr_ = create_subscription(get_node_pointer(), "sensor_msgs::msg::Image", subscribe_topic_name_, topic_qos, std::bind(&ROSTypeImagePubSubMono::callbackSubscribe, this, std::placeholders::_1));
    if (subscriber_ptr_ == 0)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        return false;
    }

    return true;
}

void ROSTypeImagePubSubMono::callbackSubscribe(void *message)
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

    int width = my_message->width();
    int height = my_message->height();
    cv::Mat cv_image(height, width, CV_8UC3, my_message->data().data());

    cv::Mat gray_image;
    cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);

    gray_msg_->width(width);
    gray_msg_->height(height);
    gray_msg_->encoding("mono8");
    gray_msg_->step(gray_image.step);
    gray_msg_->data(std::vector<uint8_t>(gray_image.data, gray_image.data + gray_image.total() * gray_image.elemSize()));
    
    publish(reinterpret_cast<intptr_t>(publisher_ptr_), gray_msg_.get());
}