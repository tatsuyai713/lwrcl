#ifndef DDS_MESSAGE_TYPE_CLASS_H
#define DDS_MESSAGE_TYPE_CLASS_H

#include <fastdds/dds/topic/TypeSupport.hpp>

namespace rcl_like_wrapper {

class MessageType {
public:
    MessageType() = default;

    explicit MessageType(eprosima::fastdds::dds::TopicDataType* message_type)
        : type_support_(eprosima::fastdds::dds::TypeSupport(message_type)) {}

    MessageType(const MessageType& other) = delete;
    MessageType& operator=(const MessageType& other) = delete;
    
    MessageType(MessageType&& other) noexcept = default;
    MessageType& operator=(MessageType&& other) noexcept = default;

    ~MessageType() = default;

    eprosima::fastdds::dds::TypeSupport get_type_support() const {
        return type_support_;
    }

private:
    eprosima::fastdds::dds::TypeSupport type_support_; //
};

#define FAST_DDS_CUSTOM_CLASS(NAMESPACE, TYPE)                      \
    namespace NAMESPACE {                                           \
        class TYPE##Type : public MessageType, public TYPE {        \
        public:                                                     \
            TYPE##Type()                                            \
                : MessageType(new TYPE##PubSubType()), TYPE() {}    \
        };                                                          \
    }

} // namespace rcl_like_wrapper

#endif // DDS_MESSAGE_TYPE_CLASS_H
