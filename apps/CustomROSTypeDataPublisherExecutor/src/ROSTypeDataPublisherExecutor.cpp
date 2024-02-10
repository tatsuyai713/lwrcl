#include "ROSTypeDataPublisherExecutor.hpp"
#include <iostream>
#include <chrono>

ROSTypeDataPublisherExecutor::ROSTypeDataPublisherExecutor()
    : RCLWNode(), topic_name_("default_topic"), interval_ms_(1000) {

    // Initialization of custom publisher subtype
    custom_pubsubtype_ = std::make_unique<CustomMessagePubSubType>();
    message_types_["CustomMessage"] = MessageType(custom_pubsubtype_.get());
    image_pubsubtype_ = std::make_unique<sensor_msgs::msg::ImagePubSubType>();
    message_types_["sensor_msgs::msg::Image"] = MessageType(image_pubsubtype_.get());
    rcl_like_wrapper_init(message_types_);

    // Create shared pointer for publishing messages
    publish_msg_ = std::make_shared<CustomMessage>();
}

ROSTypeDataPublisherExecutor::~ROSTypeDataPublisherExecutor() {
    RCLWNode::~RCLWNode();
}

bool ROSTypeDataPublisherExecutor::init(const std::string& config_file_path) {
    // Load configuration from YAML file
    YAML::Node node = YAML::LoadFile(config_file_path);
    YAML::Node config = node["config"];

    domain_number_ = config["domain_number"].as<uint8_t>(0);
    std::cout << "Domain number: " << static_cast<int>(domain_number_) << std::endl;

    for (const auto& topic_node : config["topics"]) {
        topic_name_ = topic_node["name"].as<std::string>("default_topic");
        interval_ms_ = topic_node["interval_ms"].as<uint16_t>(1000);
        std::cout << "Topic name: " << topic_name_ << ", Interval: " << interval_ms_ << " ms" << std::endl;
    }

    if (interval_ms_ == 0) {
        std::cerr << "Interval Time Error!" << std::endl;
        return false;
    }

    // Create a node, publisher, and timer
    node_ptr_ = create_node(domain_number_);
    if (!node_ptr_) {
        std::cerr << "Error: Failed to create a node." << std::endl;
        return false;
    }

    dds::TopicQos topic_qos = dds::TOPIC_QOS_DEFAULT;
    publisher_ptr_ = create_publisher(node_ptr_, "CustomMessage", topic_name_, topic_qos);
    if (!publisher_ptr_) {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        destroy_node(node_ptr_);
        return false;
    }

    // Setup timer for periodic callback
    timer_callback_ = [this]() { this->callbackPublish(interval_ms_); };
    timer_ptr_ = create_timer(node_ptr_, std::chrono::milliseconds(interval_ms_), timer_callback_);
    if (!timer_ptr_) {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        destroy_publisher(publisher_ptr_);
        destroy_node(node_ptr_);
        return false;
    }

    return true;
}

void ROSTypeDataPublisherExecutor::callbackPublish(int test) {
    // Update and publish message
    publish_msg_->index(publish_msg_->index() + 1);
    std::string s = "BigData" + std::to_string(publish_msg_->index() % 10);
    publish_msg_->message(s);

    if (!publisher_ptr_) {
        std::cerr << "Error: Invalid publisher pointer." << std::endl;
        return;
    }

    publish(reinterpret_cast<intptr_t>(publisher_ptr_), publish_msg_.get());
}