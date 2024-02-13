#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "channel.hpp"
#include "rcl_like_wrapper.hpp"

namespace rcl_like_wrapper
{

  class SubscriptionCallback
  {
  public:
    SubscriptionCallback(MessageType &message_type, std::function<void(void *)> callback_function, std::shared_ptr<void> message)
        : message_type_(message_type), callback_function_(callback_function),
          message_(message) {}

    ~SubscriptionCallback() = default;

    void invoke()
    {
      try
      {
        callback_function_(message_.get());
      }
      catch (const std::exception &e)
      {
        std::cerr << "Exception during callback invocation: " << e.what() << std::endl;
      }
      catch (...)
      {
        std::cerr << "Unknown exception during callback invocation." << std::endl;
      }
    }

  private:
    MessageType &message_type_;
    std::function<void(void *)> callback_function_;
    std::shared_ptr<void> message_;
  };

  class SubscriberListener : public dds::DataReaderListener
  {
  public:
    SubscriberListener(MessageType &message_type, std::function<void(void *)> callback_function, Channel<SubscriptionCallback *> &channel)
        : message_type_(message_type), callback_function_(callback_function), channel_(channel) {}

    void on_subscription_matched(dds::DataReader *, const dds::SubscriptionMatchedStatus &status) override
    {
      count = status.current_count;
    }

    void on_data_available(dds::DataReader *reader) override
    {
      std::shared_ptr<void> message(message_type_.type_support.create_data(),
                                    [this](void *data)
                                    { this->message_type_.type_support.delete_data(data); });
      if (reader->take_next_sample(message.get(), &sample_info_) == ReturnCode_t::RETCODE_OK && sample_info_.valid_data)
      {
        channel_.produce(new SubscriptionCallback(message_type_, callback_function_, message));
      }
    }

    std::atomic<int32_t> count{0};

  private:
    dds::SampleInfo sample_info_;
    MessageType &message_type_;
    Channel<SubscriptionCallback *> &channel_;
    std::function<void(void *)> callback_function_;
  };

  class Subscriber
  {
  public:
    Subscriber(dds::DomainParticipant *participant, MessageType &message_type, const std::string &topic,
               const dds::TopicQos &qos, std::function<void(void *)> callback_function,
               Channel<SubscriptionCallback *> &channel)
        : participant_(participant),
          listener_(message_type, callback_function, channel)
    {
      if (message_type.type_support.register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        throw std::runtime_error("Failed to register message type");
      }
      topic_ = participant_->create_topic(topic, message_type.type_support.get_type_name(), qos);
      if (!topic_)
      {
        throw std::runtime_error("Failed to create topic");
      }
      subscriber_ = participant_->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT);
      if (!subscriber_)
      {
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create subscriber");
      }
      dds::DataReaderQos reader_qos = dds::DATAREADER_QOS_DEFAULT;
      reader_qos.endpoint().history_memory_policy = rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      reader_qos.data_sharing().off();
      reader_ = subscriber_->create_datareader(topic_, reader_qos, &listener_);
      if (!reader_)
      {
        participant_->delete_subscriber(subscriber_);
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create datareader");
      }
    }

    ~Subscriber()
    {
      if (reader_)
      {
        subscriber_->delete_datareader(reader_);
      }
      if (subscriber_)
      {
        participant_->delete_subscriber(subscriber_);
      }
      if (topic_)
      {
        participant_->delete_topic(topic_);
      }
    }

    int32_t get_publisher_count()
    {
      return listener_.count.load();
    }

  private:
    dds::DomainParticipant *participant_;
    dds::Topic *topic_;
    dds::Subscriber *subscriber_;
    dds::DataReader *reader_;
    SubscriberListener listener_;
  };

} // namespace rcl_like_wrapper
