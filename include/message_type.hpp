#pragma once

#include <fastdds/dds/topic/TypeSupport.hpp>
#include <unordered_map>


namespace rcl_like_wrapper {

class MessageType {
public:
    MessageType(eprosima::fastdds::dds::TopicDataType *message_type);
    MessageType(const MessageType& other);
    MessageType& operator=(const MessageType& other);
    MessageType();
    ~MessageType();

    eprosima::fastdds::dds::TypeSupport type_support;
};

// Alias for a map that associates message type names (strings) with MessageType instances.
using MessageTypes = std::unordered_map<std::string, MessageType>;

} // namespace rcl_like_wrapper
