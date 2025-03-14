#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iostream>
#include <mutex>

#include "fast_dds_header.hpp"
#include "qos.hpp"
#include "channel.hpp"

namespace lwrcl
{

  extern std::atomic<bool> global_stop_flag;
  struct MessageInfo
  {
    std::chrono::system_clock::time_point source_timestamp;
    bool from_intra_process = false;
  };
  enum class WaitResultKind
  {
    Ready,
    Timeout,
    Error
  };
  class WaitResult
  {
  public:
    explicit WaitResult(WaitResultKind k)
        : kind_(k) {}
    WaitResultKind kind() const { return kind_; }

  private:
    WaitResultKind kind_;
  };
  struct SubscriptionHolder
  {
    std::function<bool()> has_message;
    std::function<bool(void *, MessageInfo &)> take;
  };

  #define LWRCL_SUBSCRIBE_BUFFER_SIZE 10
  template <typename T>
  class SubscribeDataBuffer
  {
  public:
    SubscribeDataBuffer()
    {
      data_wp = 0;
      data_rp = 0;
    }
    ~SubscribeDataBuffer() = default;

    T data_buffer[LWRCL_SUBSCRIBE_BUFFER_SIZE];
    int data_wp;
    int data_rp;
  };

  template <typename T>
  class SubscriberCallback : public ChannelCallback
  {
  public:
    SubscriberCallback(std::function<void(std::shared_ptr<T>)> callback_function, SubscribeDataBuffer<T> * message_buffer, std::mutex *lwrcl_subscriber_mutex_ptr)
        : callback_function_(callback_function), message_buffer_(message_buffer), lwrcl_subscriber_mutex_ptr_(lwrcl_subscriber_mutex_ptr) {}

    ~SubscriberCallback() = default;

    void invoke() override
    {
      try
      {
        std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_ptr_);
        if(message_buffer_->data_rp != message_buffer_->data_wp) {
          auto message_ptr = std::shared_ptr<T>(&message_buffer_->data_buffer[message_buffer_->data_rp], [](T *) {});
          callback_function_(message_ptr);
          message_buffer_->data_rp++;
          message_buffer_->data_rp %= LWRCL_SUBSCRIBE_BUFFER_SIZE;
        }
        else
        {
          std::cerr << "Error: Vector is empty" << std::endl;
        }
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
    std::function<void(std::shared_ptr<T>)> callback_function_;
    SubscribeDataBuffer<T> * message_buffer_;
    mutable std::mutex *lwrcl_subscriber_mutex_ptr_;
  };

  template <typename T>
  class SubscriberWaitSet
  {
  public:
    SubscriberWaitSet(std::function<void(std::shared_ptr<T>)> callback_function, Channel<ChannelCallback *>::SharedPtr channel)
        : callback_function_(callback_function), channel_(channel)
    {
      subscription_callback_ = std::make_unique<SubscriberCallback<T>>(callback_function_, &message_buffer_, &lwrcl_subscriber_mutex_);
    }

    ~SubscriberWaitSet()
    {
      stop();
    }

    void start(eprosima::fastdds::dds::DataReader *reader)
    {
      reader_ = reader;
      stop_flag_.store(false);

      auto &status_cond = reader_->get_statuscondition();

      status_cond.set_enabled_statuses(eprosima::fastdds::dds::StatusMask::data_available());

      // Attach the StatusCondition to the WaitSet
      wait_set_.attach_condition(status_cond);

      waitset_thread_ = std::thread(&SubscriberWaitSet::run, this);
    }

    void stop()
    {
      stop_flag_.store(true);
      if (waitset_thread_.joinable())
      {
        waitset_thread_.join();
      }
    }

    int32_t get_publisher_count()
    {
      return count_.load();
    }

  public:
    bool take(T &out_msg, lwrcl::MessageInfo &info)
    {
      std::lock_guard<std::mutex> lock(lwrcl_subscriber_mutex_);
      if (pollable_buffer_.empty())
      {
        return false;
      }
      auto &front = pollable_buffer_.front();
      out_msg = *(front.first);
      info = front.second;
      pollable_buffer_.erase(pollable_buffer_.begin());
      message_buffer_.data_rp++;
      message_buffer_.data_rp %= LWRCL_SUBSCRIBE_BUFFER_SIZE;
      return true;
    }

    bool has_message() const
    {
      std::lock_guard<std::mutex> lock(lwrcl_subscriber_mutex_);
      return !pollable_buffer_.empty();
    }

  private:
    void run()
    {

      while (!stop_flag_.load())
      {
        eprosima::fastdds::dds::ConditionSeq active_conditions;
        eprosima::fastrtps::Duration_t timeout{1, 0}; // Wait for 1 second

        if (wait_set_.wait(active_conditions, timeout) == ReturnCode_t::RETCODE_OK)
        {
          // printf("Data available : Time %ld\n", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
          for (auto condition : active_conditions)
          {

            eprosima::fastdds::dds::StatusCondition *status_cond = dynamic_cast<eprosima::fastdds::dds::StatusCondition *>(condition);
            if (nullptr != status_cond)
            {
              eprosima::fastdds::dds::Entity *entity = status_cond->get_entity();
              eprosima::fastdds::dds::StatusMask changed_statuses = entity->get_status_changes();

              if (changed_statuses.is_active(eprosima::fastdds::dds::StatusMask::subscription_matched())) {
                eprosima::fastdds::dds::SubscriptionMatchedStatus status;
                if (reader_->get_subscription_matched_status(status) == ReturnCode_t::RETCODE_OK)
                {
                  count_.store(status.total_count);
                }

              } else if (changed_statuses.is_active(eprosima::fastdds::dds::StatusMask::data_available()))
              {

                while (reader_->take_next_sample(&message_buffer_.data_buffer[message_buffer_.data_wp], &info_) == ReturnCode_t::RETCODE_OK)
                {
                  if (info_.valid_data)
                  {
                    std::lock_guard<std::mutex> lock(lwrcl_subscriber_mutex_);
                    if((message_buffer_.data_wp + 1) % LWRCL_SUBSCRIBE_BUFFER_SIZE == message_buffer_.data_rp) {
                      std::cerr << "Error: Data Lost" << std::endl;
                      continue;
                    }
                    message_buffer_.data_wp++;
                    message_buffer_.data_wp %= LWRCL_SUBSCRIBE_BUFFER_SIZE;
                    channel_->produce(subscription_callback_.get());
                    auto data_ptr = &message_buffer_.data_buffer[message_buffer_.data_wp];
                    lwrcl::MessageInfo new_info;
                    new_info.source_timestamp = std::chrono::system_clock::now();
                    new_info.from_intra_process = false;
                    pollable_buffer_.emplace_back(data_ptr, new_info);
                  }
                  else
                  {
                    std::cerr << "Error: Invalid data" << std::endl;
                    break;
                  }
                }
              }
            }
          }
        }
      }
    }

    std::function<void(std::shared_ptr<T>)> callback_function_;
    Channel<ChannelCallback *>::SharedPtr channel_;
    SubscribeDataBuffer<T> message_buffer_;
    std::unique_ptr<SubscriberCallback<T>> subscription_callback_;
    eprosima::fastdds::dds::SampleInfo sample_info_;
    eprosima::fastdds::dds::DataReader *reader_ = nullptr;
    std::atomic<bool> stop_flag_{false};
    std::thread waitset_thread_;
    std::atomic<int32_t> count_{0};
    eprosima::fastdds::dds::WaitSet wait_set_;
    T data_;
    eprosima::fastdds::dds::SampleInfo info_;
    mutable std::mutex lwrcl_subscriber_mutex_;
    std::vector<std::pair<T*, lwrcl::MessageInfo>> pollable_buffer_;
  };

  class ISubscription
  {
  public:
    virtual ~ISubscription() = default;
    virtual int32_t get_publisher_count() = 0;
    virtual void stop() = 0;
  };

  template <typename T>
  class Subscription : public ISubscription, public std::enable_shared_from_this<Subscription<T>>
  {
  public:
    Subscription(eprosima::fastdds::dds::DomainParticipant *participant, const std::string &topic,
                 const QoS &qos, std::function<void(std::shared_ptr<T>)> callback_function,
                 Channel<ChannelCallback *>::SharedPtr channel)
        : participant_(participant), waitset_(callback_function, channel), topic_(nullptr), subscriber_(nullptr), reader_(nullptr)
    {
      using ParentType = typename ParentTypeTraits<T>::Type;
      message_type_ = lwrcl::MessageType(new ParentType());
      lwrcl::dds::TopicQos topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT;
      if (message_type_.get_type_support().register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        throw std::runtime_error("Failed to register message type");
      }

      eprosima::fastdds::dds::Topic *retrieved_topic = dynamic_cast<eprosima::fastdds::dds::Topic *>(participant->lookup_topicdescription(topic));
      if (retrieved_topic == nullptr)
      {
        topic_ = participant_->create_topic(topic, message_type_.get_type_support().get_type_name(), topic_qos);
        if (!topic_)
        {
          throw std::runtime_error("Failed to create topic");
        }
        topic_owned_ = true;
      }
      else
      {
        topic_ = retrieved_topic;
        topic_owned_ = false;
      }

      subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
      if (!subscriber_)
      {
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create subscriber");
      }
      eprosima::fastdds::dds::DataReaderQos reader_qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
      reader_qos.endpoint().history_memory_policy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
      reader_qos.history().depth = qos.get_depth();
      if (qos.get_history() == QoS::HistoryPolicy::KEEP_ALL)
      {
        reader_qos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      }
      else
      {
        reader_qos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      }
      if (qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT)
      {
        reader_qos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      }
      else
      {
        reader_qos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      }
      if (qos.get_durability() == QoS::DurabilityPolicy::VOLATILE)
      {
        reader_qos.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      }
      else
      {
        reader_qos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      }
      reader_qos.data_sharing().automatic();
      reader_qos.properties().properties().emplace_back("fastdds.intraprocess_delivery", "true");

      reader_ = subscriber_->create_datareader(topic_, reader_qos, nullptr);
      if (!reader_)
      {
        participant_->delete_subscriber(subscriber_);
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create datareader");
      }

      waitset_.start(reader_);
    }

    ~Subscription()
    {
      waitset_.stop();
      if (reader_ != nullptr)
      {
        subscriber_->delete_datareader(reader_);
      }
      if (subscriber_ != nullptr)
      {
        participant_->delete_subscriber(subscriber_);
      }
      if (topic_ != nullptr && topic_owned_)
      {
        participant_->delete_topic(topic_);
      }
    }

    int32_t get_publisher_count()
    {
      return waitset_.get_publisher_count();
    }
    using SharedPtr = std::shared_ptr<Subscription<T>>;

    void stop()
    {
      waitset_.stop();
    }

  public:
    bool take(T &out_msg, lwrcl::MessageInfo &info)
    {
      return waitset_.take(out_msg, info);
    }

    bool has_message() const
    {
      return waitset_.has_message();
    }

  private:
    eprosima::fastdds::dds::DomainParticipant *participant_;
    SubscriberWaitSet<T> waitset_;
    eprosima::fastdds::dds::Topic *topic_;
    eprosima::fastdds::dds::Subscriber *subscriber_;
    eprosima::fastdds::dds::DataReader *reader_;
    lwrcl::MessageType message_type_;
    bool topic_owned_;
  };
  
  class WaitSet
  {
  public:
    WaitSet() = default;

    template <typename T>
    WaitSet(std::initializer_list<std::shared_ptr<Subscription<T>>> init_list)
    {
      for (auto & sub : init_list) {
        add_subscription<T>(sub);
      }
    }

template <typename T>
    void add_subscription(std::shared_ptr<Subscription<T>> sub)
    {
      SubscriptionHolder holder;
      holder.has_message = [sub]() -> bool {
        return sub->has_message();
      };
      holder.take = [sub](void * data_ptr, MessageInfo & info) -> bool {
        T* typed_ptr = static_cast<T*>(data_ptr);
        return sub->take(*typed_ptr, info);
      };
      subs_.push_back(holder);
    }

    WaitResult wait()
    {
      return wait_for(std::chrono::nanoseconds(-1));
    }

    template <typename Rep, typename Period>
    WaitResult wait(std::chrono::duration<Rep, Period> timeout)
    {
      // timeout を nanoseconds に変換
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
      return wait_for(ns);
    }

    WaitResult wait_for(std::chrono::nanoseconds timeout)
    {
      auto start = std::chrono::steady_clock::now();
      while (global_stop_flag.load() == false)
      {
        for (auto &holder : subs_)
        {
          if (holder.has_message())
          {
            return WaitResult(WaitResultKind::Ready);
          }
        }

        if (timeout.count() >= 0)
        {
          auto now = std::chrono::steady_clock::now();
          if (now - start >= timeout)
          {
            return WaitResult(WaitResultKind::Timeout);
          }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return WaitResult(WaitResultKind::Error);
    }

    const std::vector<SubscriptionHolder> &get_subscriptions() const
    {
      return subs_;
    }

  private:
    std::vector<SubscriptionHolder> subs_;
  };
} // namespace lwrcl

#endif // LWRCL_SUBSCRIBER_HPP_
