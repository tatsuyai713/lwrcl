#ifndef LWRCL_FAST_DDS_HEADER_HPP_
#define LWRCL_FAST_DDS_HEADER_HPP_

#include <unordered_map>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>

#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/core/status/StatusMask.hpp>

#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/dds/topic/TopicDescription.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastrtps/types/TypesBase.h>

#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/UDPv6TransportDescriptor.h>

#include <unordered_map>

namespace lwrcl
{
  namespace dds
  {
    using DomainParticipant = eprosima::fastdds::dds::DomainParticipant;
    using DomainParticipantFactory = eprosima::fastdds::dds::DomainParticipantFactory;
    using DomainParticipantListener = eprosima::fastdds::dds::DomainParticipantListener;
    using DomainParticipantQos = eprosima::fastdds::dds::DomainParticipantQos;
    using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;
    static const DomainParticipantQos PARTICIPANT_QOS_DEFAULT =
        eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;

    using DataReader = eprosima::fastdds::dds::DataReader;
    using DataReaderListener = eprosima::fastdds::dds::DataReaderListener;
    using DataReaderQos = eprosima::fastdds::dds::DataReaderQos;
    static const DataReaderQos DATAREADER_QOS_DEFAULT = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
    using DataWriter = eprosima::fastdds::dds::DataWriter;
    using DataWriterListener = eprosima::fastdds::dds::DataWriterListener;
    using DataWriterQos = eprosima::fastdds::dds::DataWriterQos;
    static const DataWriterQos DATAWRITER_QOS_DEFAULT = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
    using Publisher = eprosima::fastdds::dds::Publisher;
    using PublisherQos = eprosima::fastdds::dds::PublisherQos;
    static const PublisherQos PUBLISHER_QOS_DEFAULT = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
    using PublicationMatchedStatus = eprosima::fastdds::dds::PublicationMatchedStatus;
    using Subscriber = eprosima::fastdds::dds::Subscriber;

    using SubscriberQos = eprosima::fastdds::dds::SubscriberQos;
    static const SubscriberQos SUBSCRIBER_QOS_DEFAULT = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
    using SubscriptionMatchedStatus = eprosima::fastdds::dds::SubscriptionMatchedStatus;

    using SampleInfo = eprosima::fastdds::dds::SampleInfo;
    using StatusMask = eprosima::fastdds::dds::StatusMask;

    using TypeSupport = eprosima::fastdds::dds::TypeSupport;
    using Topic = eprosima::fastdds::dds::Topic;
    using RegisterdTopics = std::unordered_map<std::string, eprosima::fastdds::dds::Topic *>;
    using TopicDataType = eprosima::fastdds::dds::TopicDataType;
    using TopicDescription = eprosima::fastdds::dds::TopicDescription;
    using TopicQos = eprosima::fastdds::dds::TopicQos;
    static const TopicQos TOPIC_QOS_DEFAULT = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;

    using DomainId_t = eprosima::fastdds::dds::DomainId_t;
    using RetrunCode_t = eprosima::fastrtps::types::ReturnCode_t;

    using DurabilityQosPolicyKind_t = eprosima::fastdds::dds::DurabilityQosPolicyKind_t;
    static const DurabilityQosPolicyKind_t VOLATILE_DURABILITY_QOS =
        eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    static const DurabilityQosPolicyKind_t TRANSIENT_LOCAL_DURABILITY_QOS =
        eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    static const DurabilityQosPolicyKind_t TRANSIENT_DURABILITY_QOS =
        eprosima::fastdds::dds::TRANSIENT_DURABILITY_QOS;
    static const DurabilityQosPolicyKind_t PERSISTENT_DURABILITY_QOS =
        eprosima::fastdds::dds::PERSISTENT_DURABILITY_QOS;
    using ReliabilityQosPolicyKind = eprosima::fastdds::dds::ReliabilityQosPolicyKind;
    static const ReliabilityQosPolicyKind BEST_EFFORT_RELIABILITY_QOS =
        eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
    static const ReliabilityQosPolicyKind RELIABLE_RELIABILITY_QOS =
        eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
  }

  namespace rtps
  {
    using MemoryManagementPolicy_t = eprosima::fastrtps::rtps::MemoryManagementPolicy_t;
    using DiscoveryProtocol_t = eprosima::fastrtps::rtps::DiscoveryProtocol_t;
    static const MemoryManagementPolicy_t PREALLOCATED_WITH_REALLOC_MEMORY_MODE =
        eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  } // namespace rtps

  class MessageType
  {
  public:
    MessageType() = default;

    explicit MessageType(dds::TopicDataType *message_type)
        : type_support_(dds::TypeSupport(message_type)) {}

    MessageType(const MessageType &other) = delete;
    MessageType &operator=(const MessageType &other) = delete;

    MessageType(MessageType &&other) noexcept = default;
    MessageType &operator=(MessageType &&other) noexcept = default;

    ~MessageType() = default;

    dds::TypeSupport get_type_support() const
    {
      return type_support_;
    }

  private:
    dds::TypeSupport type_support_; //
  };

} // namespace lwrcl

#define FAST_DDS_DATA_TYPE(NAMESPACE0, NAMESPACE1, TYPE)            \
  namespace NAMESPACE0                                              \
  {                                                                 \
    namespace NAMESPACE1                                            \
    {                                                               \
      class TYPE##Type : public lwrcl::MessageType, public TYPE     \
      {                                                             \
      public:                                                       \
        TYPE##Type()                                                \
            : lwrcl::MessageType(new TYPE##PubSubType()), TYPE() {} \
      };                                                            \
    }                                                               \
  }

#endif // LWRCL_FAST_DDS_HEADER_HPP_