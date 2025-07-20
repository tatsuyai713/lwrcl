#ifndef LWRCL_FAST_DDS_HEADER_HPP_
#define LWRCL_FAST_DDS_HEADER_HPP_
#include <unordered_map>
#include <iostream>
#include <vector>
#include <functional>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>

// Fast DDS includes
#include <fastdds/dds/core/LoanableSequence.hpp>
#include <fastdds/dds/core/condition/StatusCondition.hpp>
#include <fastdds/dds/core/condition/WaitSet.hpp>
#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/core/status/PublicationMatchedStatus.hpp>
#include <fastdds/dds/core/status/StatusMask.hpp>
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/dds/topic/TopicDescription.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/qos/TopicQos.hpp>
#include <fastrtps/rtps/attributes/WriterAttributes.h>
#include <fastrtps/rtps/common/Guid.h>
#include <fastrtps/rtps/common/Time_t.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/UDPv6TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastrtps/types/TypesBase.h>
#include <fastdds/dds/core/condition/GuardCondition.hpp>

namespace lwrcl
{
  namespace dds
  {
    using DomainParticipant = eprosima::fastdds::dds::DomainParticipant;
    using DomainParticipantFactory = eprosima::fastdds::dds::DomainParticipantFactory;
    using DomainParticipantListener = eprosima::fastdds::dds::DomainParticipantListener;
    using DomainParticipantQos = eprosima::fastdds::dds::DomainParticipantQos;
    using SharedMemTransportDescriptor = eprosima::fastdds::rtps::SharedMemTransportDescriptor;

    inline const DomainParticipantQos &PARTICIPANT_QOS_DEFAULT()
    {
      static const DomainParticipantQos qos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
      return qos;
    }

    using DataReader = eprosima::fastdds::dds::DataReader;
    using DataReaderListener = eprosima::fastdds::dds::DataReaderListener;
    using DataReaderQos = eprosima::fastdds::dds::DataReaderQos;
    inline const DataReaderQos &DATAREADER_QOS_DEFAULT()
    {
      static const DataReaderQos qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
      return qos;
    }

    using DataWriter = eprosima::fastdds::dds::DataWriter;
    using DataWriterListener = eprosima::fastdds::dds::DataWriterListener;
    using DataWriterQos = eprosima::fastdds::dds::DataWriterQos;
    inline const DataWriterQos &DATAWRITER_QOS_DEFAULT()
    {
      static const DataWriterQos qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
      return qos;
    }

    using Publisher = eprosima::fastdds::dds::Publisher;
    using PublisherQos = eprosima::fastdds::dds::PublisherQos;
    inline const PublisherQos &PUBLISHER_QOS_DEFAULT()
    {
      static const PublisherQos qos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
      return qos;
    }

    using PublicationMatchedStatus = eprosima::fastdds::dds::PublicationMatchedStatus;
    using Subscriber = eprosima::fastdds::dds::Subscriber;
    using SubscriberQos = eprosima::fastdds::dds::SubscriberQos;
    inline const SubscriberQos &SUBSCRIBER_QOS_DEFAULT()
    {
      static const SubscriberQos qos = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
      return qos;
    }

    using SubscriptionMatchedStatus = eprosima::fastdds::dds::SubscriptionMatchedStatus;
    using SampleInfo = eprosima::fastdds::dds::SampleInfo;
    using StatusMask = eprosima::fastdds::dds::StatusMask;

    using TypeSupport = eprosima::fastdds::dds::TypeSupport;
    using Topic = eprosima::fastdds::dds::Topic;
    using RegisterdTopics = std::unordered_map<std::string, eprosima::fastdds::dds::Topic *>;
    using TopicDataType = eprosima::fastdds::dds::TopicDataType;
    using TopicDescription = eprosima::fastdds::dds::TopicDescription;
    using TopicQos = eprosima::fastdds::dds::TopicQos;
    inline const TopicQos &TOPIC_QOS_DEFAULT()
    {
      static const TopicQos qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
      return qos;
    }

    using DomainId_t = eprosima::fastdds::dds::DomainId_t;
    using ReturnCode_t = eprosima::fastrtps::types::ReturnCode_t;

    using DurabilityQosPolicyKind_t = eprosima::fastdds::dds::DurabilityQosPolicyKind_t;
    inline DurabilityQosPolicyKind_t VOLATILE_DURABILITY_QOS()
    {
      return eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
    }
    inline DurabilityQosPolicyKind_t TRANSIENT_LOCAL_DURABILITY_QOS()
    {
      return eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    }
    inline DurabilityQosPolicyKind_t TRANSIENT_DURABILITY_QOS()
    {
      return eprosima::fastdds::dds::TRANSIENT_DURABILITY_QOS;
    }
    inline DurabilityQosPolicyKind_t PERSISTENT_DURABILITY_QOS()
    {
      return eprosima::fastdds::dds::PERSISTENT_DURABILITY_QOS;
    }
    using ReliabilityQosPolicyKind = eprosima::fastdds::dds::ReliabilityQosPolicyKind;
    inline ReliabilityQosPolicyKind BEST_EFFORT_RELIABILITY_QOS()
    {
      return eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
    }
    inline ReliabilityQosPolicyKind RELIABLE_RELIABILITY_QOS()
    {
      return eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    }
  }

  namespace rtps
  {
    using MemoryManagementPolicy_t = eprosima::fastrtps::rtps::MemoryManagementPolicy_t;
    using DiscoveryProtocol_t = eprosima::fastrtps::rtps::DiscoveryProtocol_t;
    static const MemoryManagementPolicy_t PREALLOCATED_WITH_REALLOC_MEMORY_MODE =
        eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  } // namespace rtps

  class MessageType : public std::enable_shared_from_this<MessageType>
  {
  public:
    using SharedPtr = std::shared_ptr<MessageType>;
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

template <typename T>
struct ParentTypeTraits;

#endif // LWRCL_FAST_DDS_HEADER_HPP_
