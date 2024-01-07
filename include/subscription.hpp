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

namespace rcl_like_wrapper {

class SubscriptionCallback {
public:
  SubscriptionCallback(MessageType &message_type, std::function<void(void*)> callback_function, void* message)
      : message_type_(message_type), callback_function_(callback_function),
        message_(message) {}

  ~SubscriptionCallback() {
    message_type_.type_support.delete_data(message_);
  }

  void invoke() {
    callback_function_(message_);
  }

private:
  MessageType &message_type_;
  std::function<void(void*)> callback_function_;
  void* message_;
};

class SubscriptionListener : public dds::DataReaderListener {
public:
  SubscriptionListener(MessageType &message_type, std::function<void(void*)> callback_function, Channel<SubscriptionCallback *> &channel)
      : message_type_(message_type), callback_function_(callback_function), channel_(channel) {}

  void on_subscription_matched(dds::DataReader *, const dds::SubscriptionMatchedStatus &status) override {
    count = status.current_count;
  }

  void on_data_available(dds::DataReader *reader) override {
    void* message = message_type_.type_support.create_data();
    if (reader->take_next_sample(message, &sample_info_) == ReturnCode_t::RETCODE_OK && sample_info_.valid_data) {
      channel_.produce(new SubscriptionCallback(message_type_, callback_function_, message));
    }
  }

  std::atomic<int32_t> count{0};

private:
  dds::SampleInfo sample_info_;
  MessageType &message_type_;
  Channel<SubscriptionCallback *> &channel_;
  std::function<void(void*)> callback_function_;
};

class Subscription {
public:
  Subscription(dds::DomainParticipant *participant, MessageType &message_type, const std::string &topic,
               const dds::TopicQos &qos, std::function<void(void*)> callback_function,
               Channel<SubscriptionCallback *> &channel)
      : participant_(participant),
        listener_(message_type, callback_function, channel) {
    message_type.type_support.register_type(participant_);
    topic_ = participant_->create_topic(topic, message_type.type_support.get_type_name(), qos);
    subscriber_ = participant_->create_subscriber(dds::SUBSCRIBER_QOS_DEFAULT);

    dds::DataReaderQos reader_qos = dds::DATAREADER_QOS_DEFAULT;
    reader_qos.endpoint().history_memory_policy = rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    reader_qos.data_sharing().off();
    reader_ = subscriber_->create_datareader(topic_, reader_qos, &listener_);
  }

  void destroy() {
    delete this;
  }

  int32_t get_publisher_count() {
    return listener_.count;
  }

private:
  ~Subscription() {
    subscriber_->delete_datareader(reader_);
    participant_->delete_subscriber(subscriber_);
    participant_->delete_topic(topic_);
  }

  dds::DomainParticipant *participant_;
  dds::Topic *topic_;
  dds::Subscriber *subscriber_;
  dds::DataReader *reader_;
  SubscriptionListener listener_;
};

} // namespace rcl_like_wrapper
