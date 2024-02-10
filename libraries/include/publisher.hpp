#pragma once

#include <atomic>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "rcl_like_wrapper.hpp"

namespace rcl_like_wrapper
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

  class Publisher
  {
  public:
    Publisher(dds::DomainParticipant *participant, MessageType &message_type, const std::string &topic,
              const dds::TopicQos &qos)
        : participant_(participant), message_type_(message_type)
    {
      message_type_.type_support.register_type(participant_);
      topic_ = participant_->create_topic(topic, message_type_.type_support.get_type_name(), qos);
      publisher_ = participant_->create_publisher(dds::PUBLISHER_QOS_DEFAULT);

      dds::DataWriterQos writer_qos = dds::DATAWRITER_QOS_DEFAULT;
      writer_qos.endpoint().history_memory_policy = rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      writer_qos.publish_mode().kind = dds::ASYNCHRONOUS_PUBLISH_MODE;
      writer_qos.data_sharing().off();
      writer_ = publisher_->create_datawriter(topic_, writer_qos, &listener_);
    }

    ~Publisher()
    {
      publisher_->delete_datawriter(writer_);
      participant_->delete_publisher(publisher_);
      participant_->delete_topic(topic_);
    }
    
    void publish(void *message) const
    {
      writer_->write(message);
      message_type_.type_support.delete_data(message);
    }

    int32_t get_subscriber_count()
    {
      return listener_.count;
    }

  private:
    dds::DomainParticipant *participant_;
    dds::Topic *topic_;
    dds::Publisher *publisher_;
    dds::DataWriter *writer_;
    PublisherListener listener_;
    MessageType &message_type_;
  };

} // namespace rcl_like_wrapper
