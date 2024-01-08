#pragma once

#include <forward_list>
#include <memory>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include "channel.hpp"
#include "eprosima_namespace.hpp"
#include "publisher.hpp"
#include "rcl_like_wrapper.hpp"
#include "subscriber.hpp"

namespace rcl_like_wrapper {

class Node {
public:
  Node(uint16_t domain_id) {
    participant_ =
        dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, dds::PARTICIPANT_QOS_DEFAULT);
  }

  void destroy() {
    for (auto *publisher : publisher_list_) {
      delete publisher;
    }
    for (auto *subscription : subscription_list_) {
      subscription->destroy();
    }
    delete this;
  }

  void spin() {
    SubscriptionCallback* callback;
    while (channel_.consume(callback)) {
      callback->invoke();
      delete callback;
    }
  }

  void spin_once() {
    SubscriptionCallback* callback;
    if (channel_.consume_nowait(callback)) {
      callback->invoke();
      delete callback;
    }
  }

  void stop_spin() {
    channel_.close();
  }

  Publisher *create_publisher(MessageType &message_type, const std::string &topic, const dds::TopicQos &qos) {
    publisher_list_.emplace_front(new Publisher(participant_, message_type, topic, qos));
    return publisher_list_.front();
  }

  Subscriber *create_subscriber(MessageType &message_type, const std::string &topic, const dds::TopicQos &qos,
                                    std::function<void(void*)> callback_function) {
    subscription_list_.emplace_front(new Subscriber(participant_, message_type, topic, qos, callback_function, channel_));
    return subscription_list_.front();
  }

private:
  ~Node() {
    dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
  }

  dds::DomainParticipant *participant_;
  std::forward_list<Publisher *> publisher_list_;
  std::forward_list<Subscriber *> subscription_list_;
  Channel<SubscriptionCallback*> channel_;
};

} // namespace rcl_like_wrapper
