#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <atomic>
#include <string>

#include "fast_dds_header.hpp"
#include "qos.hpp"
namespace lwrcl
{

  class PublisherListener : public dds::DataWriterListener
  {
  public:
    void on_publication_matched(dds::DataWriter *, const dds::PublicationMatchedStatus &status) override
    {
      count = status.current_count;
    }

    std::atomic<int32_t> count{0};
  };

  class IPublisher
  {
  public:
    virtual ~IPublisher() = default;
    virtual int32_t get_subscriber_count() = 0;
  };
  template <typename T>
  class Publisher : public IPublisher, public std::enable_shared_from_this<Publisher<T>>
  {
  public:
    Publisher(dds::DomainParticipant *participant, const std::string &topic_name,
              const QoS &qos)
        : participant_(participant), topic_(nullptr), publisher_(nullptr), writer_(nullptr)
    {
      lwrcl::dds::TopicQos topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT();

      using ParentType = typename ParentTypeTraits<T>::Type;
      message_type_ = lwrcl::MessageType(new ParentType());

      if (message_type_.get_type_support().register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        throw std::runtime_error("Failed to register message type");
      }

      std::string type_name = message_type_.get_type_support().get_type_name();

      publisher_ = participant_->create_publisher(dds::PUBLISHER_QOS_DEFAULT());
      if (!publisher_)
      {
        throw std::runtime_error("Failed to create publisher");
      }
      dds::Topic *retrieved_topic = dynamic_cast<eprosima::fastdds::dds::Topic *>(participant->lookup_topicdescription(topic_name));
      if (retrieved_topic == nullptr)
      {
        topic_ = participant_->create_topic(topic_name, type_name, topic_qos);
        if (!topic_)
        {
          participant_->delete_publisher(publisher_);  // Cleanup on failure
          throw std::runtime_error("Failed to create topic");
        }
        topic_owned_ = true;
      }
      else
      {
        topic_ = retrieved_topic;
        topic_owned_ = false;
      }

      dds::DataWriterQos writer_qos = dds::DATAWRITER_QOS_DEFAULT();
      writer_qos.endpoint().history_memory_policy = rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      // writer_qos.data_sharing().automatic();
      writer_qos.history().depth = qos.get_depth();
      if (qos.get_history() == QoS::HistoryPolicy::KEEP_ALL)
      {
        writer_qos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      }
      else
      {
        writer_qos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      }
      if (qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT)
      {
        writer_qos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      }
      else
      {
        writer_qos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      }
      if (qos.get_durability() == QoS::DurabilityPolicy::VOLATILE)
      {
        writer_qos.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      }
      else
      {
        writer_qos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      }
      writer_qos.data_sharing().automatic();
      // writer_qos.data_sharing().on("shared_directory");
      writer_qos.properties().properties().emplace_back("fastdds.intraprocess_delivery", "true");
      writer_ = publisher_->create_datawriter(topic_, writer_qos, &listener_, eprosima::fastdds::dds::StatusMask::all());
      if (!writer_)
      {
        participant_->delete_publisher(publisher_); // Cleanup on failure
        participant_->delete_topic(topic_);         // Cleanup on failure
        throw std::runtime_error("Failed to create datawriter");
      }
    }

    ~Publisher()
    {
      if (writer_ != nullptr)
      {
        publisher_->delete_datawriter(writer_);
      }
      if (publisher_ != nullptr)
      {
        participant_->delete_publisher(publisher_);
      }
      if (topic_ != nullptr && topic_owned_)
      {
        participant_->delete_topic(topic_);
      }
    }

    void publish(std::shared_ptr<T> message) const
    {
      writer_->write(message.get());
    }

    void publish(T & message) const
    {
      writer_->write(&message);
    }

    int32_t get_subscriber_count()
    {
      return listener_.count;
    }

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    dds::DomainParticipant *participant_;
    MessageType message_type_;
    dds::Topic *topic_;
    dds::Publisher *publisher_;
    dds::DataWriter *writer_;
    PublisherListener listener_;
    bool topic_owned_;
  };
} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_