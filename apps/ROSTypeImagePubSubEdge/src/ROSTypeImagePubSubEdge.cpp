#include "ROSTypeImagePubSubEdge.hpp"
#include <iostream>
#include <chrono>

ROSTypeImagePubSubEdge::ROSTypeImagePubSubEdge(uint16_t domain_number)
    : RCLWNode(domain_number), publish_topic_name_("default_topic"), subscribe_topic_name_("default_topic"), interval_ms_(1000) {

    std::unique_ptr<sensor_msgs::msg::ImagePubSubType> image_pubsubtype = std::make_unique<sensor_msgs::msg::ImagePubSubType>();
    message_types_["sensor_msgs::msg::Image"] = MessageType(image_pubsubtype.release());
    rcl_like_wrapper_init(message_types_);

    counter_ = 0;
}

ROSTypeImagePubSubEdge::~ROSTypeImagePubSubEdge() {
}

bool ROSTypeImagePubSubEdge::init(const std::string& config_file_path) {
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

    subscriber_ptr_ = create_subscription(get_node_pointer(), "sensor_msgs::msg::Image", subscribe_topic_name_, topic_qos, std::bind(&ROSTypeImagePubSubEdge::callbackSubscribe, this, std::placeholders::_1));
    if (subscriber_ptr_ == 0)
    {
        std::cerr << "Error: Failed to create a subscription." << std::endl;
        return false;
    }

    return true;
}

void ROSTypeImagePubSubEdge::callbackSubscribe(void *message)
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

    cv::Mat gray_image(height, width, CV_8UC1, my_message->data().data());

    cv::Mat edges;
    cv::Canny(gray_image, edges, 50, 150);

    std::unique_ptr<sensor_msgs::msg::Image> edge_msg = std::make_unique<sensor_msgs::msg::Image>();
    edge_msg->width(width);
    edge_msg->height(height);
    edge_msg->encoding("mono8");
    edge_msg->step(edges.step);
    edge_msg->data(std::vector<uint8_t>(edges.data, edges.data + edges.total() * edges.elemSize()));

    publish(reinterpret_cast<intptr_t>(publisher_ptr_), edge_msg.release());
}
