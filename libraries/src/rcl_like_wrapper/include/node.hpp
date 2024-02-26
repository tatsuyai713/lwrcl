#pragma once

#include <forward_list>
#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <stdexcept>
#include <iostream>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include "channel.hpp"
#include "eprosima_namespace.hpp"
#include "publisher.hpp"
#include "rcl_like_wrapper.hpp"
#include "subscriber.hpp"
#include "timer.hpp"

namespace rcl_like_wrapper
{

  class Node
  {
  public:
    Node(uint16_t domain_id)
    {
      participant_ =
          dds::DomainParticipantFactory::get_instance()->create_participant(domain_id, dds::PARTICIPANT_QOS_DEFAULT);
      if (!participant_)
      {
        throw std::runtime_error("Failed to create domain participant");
      }
    }

    ~Node()
    {
      dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    void destroy()
    {
      publisher_list_.clear();
      subscription_list_.clear();
      timer_list_.clear();
    }

    Publisher *create_publisher(MessageType &message_type, const std::string &topic, const dds::TopicQos &qos)
    {
      auto publisher = std::make_unique<Publisher>(participant_, message_type, topic, qos);
      Publisher *raw_ptr = publisher.get();
      publisher_list_.push_front(std::move(publisher));
      return raw_ptr;
    }

    Subscriber *create_subscription(MessageType &message_type, const std::string &topic, const dds::TopicQos &qos,
                                    std::function<void(void *)> callback_function)
    {
      auto subscriber = std::make_unique<Subscriber>(participant_, message_type, topic, qos, callback_function, channel_);
      Subscriber *raw_ptr = subscriber.get();
      subscription_list_.push_front(std::move(subscriber));
      return raw_ptr;
    }

    Timer *create_timer(std::chrono::microseconds period, std::function<void()> callback_function)
    {
      auto timer = std::make_unique<Timer>(period, callback_function);
      Timer *raw_ptr = timer.get();
      timer_list_.push_front(std::move(timer));
      return raw_ptr;
    }

    void spin()
    {

      while (!channel_.is_closed() && rcl_like_wrapper::ok())
      {
        SubscriptionCallback *callback;
        while (channel_.consume_nowait(callback))
        {
          if (callback)
          {
            callback->invoke();
          }
        }

        auto now = std::chrono::steady_clock::now();
        for (auto &timer : timer_list_)
        {
          timer->check_and_call();
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    }

    void spin_once()
    {
      SubscriptionCallback *callback;
      if (channel_.consume_nowait(callback))
      {
        callback->invoke();
      }

      for (auto &timer : timer_list_)
      {
        timer->check_and_call();
      }
    }

    void spin_some()
    {
      bool event_processed = false;

      do
      {
        event_processed = false;

        SubscriptionCallback *callback;
        while (channel_.consume_nowait(callback))
        {
          callback->invoke();
          event_processed = true;
        }

        for (auto &timer : timer_list_)
        {
          timer->check_and_call();
        }
      } while (event_processed);
    }

    void stop_spin()
    {
      channel_.close();
    }

  private:
    dds::DomainParticipant *participant_;
    std::forward_list<std::unique_ptr<Publisher>> publisher_list_;
    std::forward_list<std::unique_ptr<Subscriber>> subscription_list_;
    std::forward_list<std::unique_ptr<Timer>> timer_list_;
    Channel<SubscriptionCallback *> channel_;
  };

} // namespace rcl_like_wrapper
