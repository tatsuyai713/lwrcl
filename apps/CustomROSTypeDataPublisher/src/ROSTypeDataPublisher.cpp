#include "ROSTypeDataPublisher.hpp"

int stop_flag = 0;
void abort_handler(int sig);

void abort_handler(int sig)
{
    stop_flag = 1;
}

ROSTypeDataPublisher::ROSTypeDataPublisher()
    : domain_number_(0), topic_name_(std::string("default_topic")), interval_ms_(1000)
{
}

ROSTypeDataPublisher::~ROSTypeDataPublisher()
{
}

bool ROSTypeDataPublisher::init(std::string config_file_path)
{
    YAML::Node node = YAML::LoadFile(config_file_path);
    YAML::Node config = node["config"];

    domain_number_ = config["domain_number"].as<uint8_t>(0);
    std::cout << "domain_number = " << int(domain_number_) << std::endl;

    YAML::Node topics = config["topics"];
    if (topics.IsSequence())
    {
        for (const auto topic_node : topics)
        {
            topic_name_ = topic_node["name"].as<std::string>("default_topic");
            interval_ms_ = topic_node["interval_ms"].as<uint16_t>(1000);
            std::cout << "name = " << topic_name_ << ", interval = " << interval_ms_ << "[ms]" << std::endl;
        }
    }

    if (interval_ms_ == 0)
    {
        std::cout << "Interval Time Error!" << std::endl;
        return false;
    }

    MessageTypes messageTypes;

    // Directly create a unique pointer
    std::unique_ptr<CustomMessagePubSubType> customPubSubType = std::make_unique<CustomMessagePubSubType>();

    // Directly create rcl_like_wrapper::MessageType with a raw pointer
    messageTypes["CustomMessage"] = MessageType(customPubSubType.get());

    rcl_like_wrapper_init(messageTypes);

    // Create a node with domain ID 0
    node_ptr_ = create_node(domain_number_);
    if (node_ptr_ == 0)
    {
        std::cerr << "Error: Failed to create a node." << std::endl;
        return false;
    }

    // Create a publisher with a topic and default QoS
    dds::TopicQos topic_qos = dds::TOPIC_QOS_DEFAULT;
    publisher_ptr_ = create_publisher(node_ptr_, "CustomMessage", topic_name_, topic_qos);
    if (publisher_ptr_ == 0)
    {
        std::cerr << "Error: Failed to create a publisher." << std::endl;
        destroy_node(node_ptr_);
        return false;
    }
    publish_msg_ = std::make_shared<CustomMessage>();

    int test = 100;
    timer_callback_ = [this, test]()
    {
        this->callbackPublish(test);
    };

    timer_ptr_ = create_timer(node_ptr_, std::chrono::milliseconds(test), timer_callback_);

    if (timer_ptr_ == 0)
    {
        std::cerr << "Error: Failed to create a timer." << std::endl;
        destroy_publisher(publisher_ptr_);
        destroy_node(node_ptr_);
        return false;
    }
    return true;
}

void ROSTypeDataPublisher::run()
{
    // Spin the node to handle incoming messages
    spin(node_ptr_);

    // Clean up
    destroy_publisher(publisher_ptr_);
    stop_spin(node_ptr_);
    destroy_node(node_ptr_);
}

void ROSTypeDataPublisher::callbackPublish(int test)
{

    // Simulate sending data periodically

    publish_msg_->index(publish_msg_->index() + 1);
    size_t data_size = publish_msg_->pose().size();
    printf("%ld\n", data_size);
    std::string s = "BigData" + std::to_string(publish_msg_->index() % 10);
    publish_msg_->message() = s;

    // Publish the message
    if (publisher_ptr_ == (intptr_t) nullptr)
    {
        std::cerr << "Error: Invalid publisher pointer." << std::endl;
        return;
    }

    publish(reinterpret_cast<intptr_t>(publisher_ptr_), publish_msg_.get());
}