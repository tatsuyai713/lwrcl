#include "message_type.hpp"

namespace rcl_like_wrapper {

MessageType::MessageType(eprosima::fastdds::dds::TopicDataType *message_type)
    : type_support(message_type) {}

MessageType::MessageType(const MessageType& other)
    : type_support(other.type_support) {}

MessageType::MessageType() : type_support(nullptr) {}

MessageType& MessageType::operator=(const MessageType& other) {
    if (this != &other) {
        type_support = other.type_support;
    }
    return *this;
}

MessageType::~MessageType() {}

} // namespace rcl_like_wrapper
