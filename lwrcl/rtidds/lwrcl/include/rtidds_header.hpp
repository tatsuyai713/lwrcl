#ifndef LWRCL_RTIDDS_HEADER_HPP_
#define LWRCL_RTIDDS_HEADER_HPP_
#include <unordered_map>
#include <iostream>
#include <vector>
#include <functional>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

#include <dds/dds.hpp>

namespace lwrcl
{
  namespace dds
  {
    // Domain Participant types
    using DomainParticipant = ::dds::domain::DomainParticipant;
    using DomainParticipantListener = ::dds::domain::DomainParticipantListener;
    using DomainParticipantQos = ::dds::domain::qos::DomainParticipantQos;

    inline ::dds::domain::qos::DomainParticipantQos PARTICIPANT_QOS_DEFAULT()
    {
      return ::dds::core::QosProvider::Default().participant_qos();
    }

    // DataReader types
    using DataReader = ::dds::sub::DataReader<void>; // Generic DataReader
    using DataReaderListener = ::dds::sub::DataReaderListener<void>;
    using DataReaderQos = ::dds::sub::qos::DataReaderQos;

    inline ::dds::sub::qos::DataReaderQos DATAREADER_QOS_DEFAULT()
    {
      return ::dds::core::QosProvider::Default().datareader_qos();
    }

    // DataWriter types
    using DataWriter = ::dds::pub::DataWriter<void>; // Generic DataWriter
    using DataWriterListener = ::dds::pub::DataWriterListener<void>;
    using DataWriterQos = ::dds::pub::qos::DataWriterQos;

    inline ::dds::pub::qos::DataWriterQos DATAWRITER_QOS_DEFAULT()
    {
      return ::dds::core::QosProvider::Default().datawriter_qos();
    }

    // Publisher types
    using Publisher = ::dds::pub::Publisher;
    using PublisherQos = ::dds::pub::qos::PublisherQos;

    inline ::dds::pub::qos::PublisherQos PUBLISHER_QOS_DEFAULT()
    {
      return ::dds::core::QosProvider::Default().publisher_qos();
    }

    // Subscriber types
    using Subscriber = ::dds::sub::Subscriber;
    using SubscriberQos = ::dds::sub::qos::SubscriberQos;

    inline ::dds::sub::qos::SubscriberQos SUBSCRIBER_QOS_DEFAULT()
    {
      return ::dds::core::QosProvider::Default().subscriber_qos();
    }

    // Status types
    using PublicationMatchedStatus = ::dds::core::status::PublicationMatchedStatus;
    using SubscriptionMatchedStatus = ::dds::core::status::SubscriptionMatchedStatus;
    using SampleInfo = ::dds::sub::SampleInfo;
    using StatusMask = ::dds::core::status::StatusMask;

    // Topic types
    using Topic = ::dds::topic::Topic<void>; // Generic Topic
    using RegisterdTopics = std::unordered_map<std::string, ::dds::topic::Topic<void>*>;
    using TopicDescription = ::dds::topic::TopicDescription;
    using TopicQos = ::dds::topic::qos::TopicQos;

    inline ::dds::topic::qos::TopicQos TOPIC_QOS_DEFAULT()
    {
      return ::dds::core::QosProvider::Default().topic_qos();
    }

    // Basic types
    using DomainId_t = uint32_t;
    using RetrunCode_t = ::dds::core::status::StatusMask;

    // QoS Policy types
    using DurabilityQosPolicyKind_t = ::dds::core::policy::Durability::Kind;

    inline DurabilityQosPolicyKind_t VOLATILE_DURABILITY_QOS()
    {
      return ::dds::core::policy::Durability::Kind::VOLATILE;
    }

    inline DurabilityQosPolicyKind_t TRANSIENT_LOCAL_DURABILITY_QOS()
    {
      return ::dds::core::policy::Durability::Kind::TRANSIENT_LOCAL;
    }

    inline DurabilityQosPolicyKind_t TRANSIENT_DURABILITY_QOS()
    {
      return ::dds::core::policy::Durability::Kind::TRANSIENT;
    }

    inline DurabilityQosPolicyKind_t PERSISTENT_DURABILITY_QOS()
    {
      return ::dds::core::policy::Durability::Kind::PERSISTENT;
    }

    using ReliabilityQosPolicyKind = ::dds::core::policy::Reliability::Kind;

    inline ReliabilityQosPolicyKind BEST_EFFORT_RELIABILITY_QOS()
    {
      return ::dds::core::policy::Reliability::Kind::BEST_EFFORT;
    }

    inline ReliabilityQosPolicyKind RELIABLE_RELIABILITY_QOS()
    {
      return ::dds::core::policy::Reliability::Kind::RELIABLE;
    }
  }

  namespace rtps
  {
    // RTI's public Modern C++ API doesn't expose the Fast DDS RTPS types.
    // These are provided for compatibility but may not have direct equivalents
    enum class MemoryManagementPolicy_t
    {
      PREALLOCATED_WITH_REALLOC_MEMORY_MODE
    };

    enum class DiscoveryProtocol_t
    {
      SIMPLE,
      EXTERNAL
    };

    static const MemoryManagementPolicy_t PREALLOCATED_WITH_REALLOC_MEMORY_MODE =
        MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  } // namespace rtps

  class MessageType : public std::enable_shared_from_this<MessageType>
  {
  public:
    using SharedPtr = std::shared_ptr<MessageType>;
    MessageType() = default;

    // Compatibility wrapper for code that carries a backend type name.
    template<typename T>
    explicit MessageType(T* message_type)
        : type_name_(typeid(T).name()) {}

    MessageType(const MessageType &other) = delete;
    MessageType &operator=(const MessageType &other) = delete;

    MessageType(MessageType &&other) noexcept = default;
    MessageType &operator=(MessageType &&other) noexcept = default;

    ~MessageType() = default;

    // RTI registers generated types through topic_type_support<T>.
    std::string get_type_name() const
    {
      return type_name_;
    }

  private:
    std::string type_name_;
  };

} // namespace lwrcl

template <typename T>
struct ParentTypeTraits;

#endif // LWRCL_RTIDDS_HEADER_HPP_
