#include "ROSTypeDataPublisherExecutor.hpp"
#include <iostream>
#include <chrono>

ROSTypeDataPublisherExecutor::ROSTypeDataPublisherExecutor(uint16_t domain_number)
    : Node(domain_number), topic_name_("default_topic"), interval_ms_(1000) {
}

ROSTypeDataPublisherExecutor::~ROSTypeDataPublisherExecutor() {
}

bool ROSTypeDataPublisherExecutor::init(const std::string& config_file_path) {
    // Load configuration from YAML file
    YAML::Node node = YAML::LoadFile(config_file_path);
    YAML::Node config = node["config"];

    for (const auto& topic_node : config["topics"]) {
        topic_name_ = topic_node["name"].as<std::string>("default_topic");
        interval_ms_ = topic_node["interval_ms"].as<uint16_t>(1000);
        std::cout << "Topic name: " << topic_name_ << ", Interval: " << interval_ms_ << " ms" << std::endl;
    }

    if (interval_ms_ <= 0) {
        std::cerr << "Interval Time Error!" << std::endl;
        return false;
    }

    lwrcl::dds::TopicQos topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
    publisher_ptr_ = create_publisher<CustomMessage>(&pub_message_type_, topic_name_, topic_qos);
    if (!publisher_ptr_) {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        return false;
    }

    // Setup timer for periodic callback
    timer_callback_ = [this]() { this->callbackPublish(interval_ms_); };
    timer_ptr_ = create_timer<std::chrono::milliseconds>(std::chrono::milliseconds(interval_ms_), timer_callback_);
    if (!timer_ptr_) {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        return false;
    }

    return true;
}

void ROSTypeDataPublisherExecutor::callbackPublish(int test) {
    // Update and publish message
    std::shared_ptr<CustomMessage> publish_msg = std::make_shared<CustomMessage>();
    publish_msg->index(publish_msg->index() + 1);
    std::string s = "BigData" + std::to_string(publish_msg->index() % 10);
    publish_msg->message(s);

    publisher_ptr_->publish(publish_msg.get());
}
