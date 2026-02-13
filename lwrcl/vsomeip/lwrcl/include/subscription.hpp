#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <vsomeip/vsomeip.hpp>
#include "qos.hpp"
#include "channel.hpp"
#include "vsomeip_header.hpp"
#include "serialized_message.hpp"

#define MAX_POLLABLE_BUFFER_SIZE 100

namespace lwrcl
{

  // Forward declarations
  template <typename T>
  class Serialization;

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
    explicit WaitResult(WaitResultKind k) : kind_(k) {}
    WaitResultKind kind() const { return kind_; }

  private:
    WaitResultKind kind_;
  };

  struct SubscriptionHolder
  {
    std::function<bool()> has_message;
    std::function<bool(void *, MessageInfo &)> take;
  };

  template <typename T>
  class SubscriberCallback : public ChannelCallback
  {
  public:
    SubscriberCallback(
        std::function<void(std::shared_ptr<T>)> callback_function, std::shared_ptr<T> message_ptr,
        std::shared_ptr<std::mutex> lwrcl_subscriber_mutex)
        : callback_function_(callback_function),
          message_ptr_(message_ptr),
          lwrcl_subscriber_mutex_(std::move(lwrcl_subscriber_mutex))
    {
    }

    ~SubscriberCallback() noexcept = default;

    SubscriberCallback(const SubscriberCallback &) = delete;
    SubscriberCallback &operator=(const SubscriberCallback &) = delete;
    SubscriberCallback(SubscriberCallback &&) = default;
    SubscriberCallback &operator=(SubscriberCallback &&) = default;

    void invoke() override
    {
      std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
      try
      {
        callback_function_(message_ptr_);
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
    std::shared_ptr<T> message_ptr_;
    std::shared_ptr<std::mutex> lwrcl_subscriber_mutex_;
  };

  // vsomeip SubscriberWaitSet - purely callback-driven (no polling thread needed)
  template <typename T>
  class SubscriberWaitSet
  {
  public:
    SubscriberWaitSet(
        std::function<void(std::shared_ptr<T>)> callback_function,
        CallbackChannel::SharedPtr channel)
        : callback_function_(callback_function),
          channel_(channel),
          stop_flag_(false),
          lwrcl_subscriber_mutex_(std::make_shared<std::mutex>()),
          pollable_buffer_(),
          count_(0)
    {
    }

    ~SubscriberWaitSet() { stop(); }

    SubscriberWaitSet(const SubscriberWaitSet &) = delete;
    SubscriberWaitSet &operator=(const SubscriberWaitSet &) = delete;
    SubscriberWaitSet(SubscriberWaitSet &&) = delete;
    SubscriberWaitSet &operator=(SubscriberWaitSet &&) = delete;

    // Called when vsomeip receives a message (from vsomeip's internal thread)
    void on_message_received(std::shared_ptr<T> message)
    {
      if (stop_flag_.load())
        return;

      lwrcl::MessageInfo new_info;
      new_info.source_timestamp = std::chrono::system_clock::now();
      new_info.from_intra_process = false;

      {
        std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
        pollable_buffer_.emplace_back(message, new_info);
        if (pollable_buffer_.size() > MAX_POLLABLE_BUFFER_SIZE)
        {
          pollable_buffer_.pop_front();
        }
      }

      channel_->produce(std::make_shared<SubscriberCallback<T>>(
          callback_function_, message, lwrcl_subscriber_mutex_));
    }

    void ready(std::shared_ptr<vsomeip::application> /*app*/)
    {
      stop_flag_.store(false);
    }

    void start()
    {
      // No polling thread needed - vsomeip is callback-driven
    }

    void stop()
    {
      stop_flag_.store(true);
    }

    int32_t get_publisher_count()
    {
      return count_.load();
    }

    void set_publisher_count(int32_t count)
    {
      count_.store(count);
    }

    bool take(std::shared_ptr<T> &out_msg, lwrcl::MessageInfo &info)
    {
      std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
      if (pollable_buffer_.empty())
      {
        return false;
      }
      auto front = std::move(pollable_buffer_.front());
      pollable_buffer_.pop_front();
      out_msg = std::move(front.first);
      info = front.second;
      return true;
    }

    bool take(T &out_msg, lwrcl::MessageInfo &info)
    {
      std::shared_ptr<T> msg_ptr;
      if (!take(msg_ptr, info))
      {
        return false;
      }
      out_msg = *msg_ptr;
      return true;
    }

    bool has_message()
    {
      std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
      return !pollable_buffer_.empty();
    }

  private:
    std::function<void(std::shared_ptr<T>)> callback_function_;
    CallbackChannel::SharedPtr channel_;
    std::atomic<bool> stop_flag_;

    std::shared_ptr<std::mutex> lwrcl_subscriber_mutex_;
    std::deque<std::pair<std::shared_ptr<T>, lwrcl::MessageInfo>> pollable_buffer_;
    std::atomic<int32_t> count_{0};
  };

  class ISubscription
  {
  public:
    virtual ~ISubscription() = default;
    virtual void stop() = 0;

  protected:
    ISubscription() = default;
  };

  template <typename T>
  class Subscription : public ISubscription, public std::enable_shared_from_this<Subscription<T>>
  {
  public:
    using SharedPtr = std::shared_ptr<Subscription<T>>;

    Subscription(
        std::shared_ptr<vsomeip::application> app, const std::string &topic_name,
        const QoS &qos, std::function<void(std::shared_ptr<T>)> callback_function,
        CallbackChannel::SharedPtr channel)
        : app_(app),
          waitset_(callback_function, channel),
          topic_name_(topic_name),
          service_id_(someip_id::topic_to_service_id(topic_name)),
          instance_id_(someip_id::default_instance_id()),
          event_id_(someip_id::default_event_id()),
          eventgroup_id_(someip_id::default_eventgroup_id())
    {
      try
      {
        waitset_.ready(app_);

        // Request the service
        app_->request_service(service_id_, instance_id_);

        // Request event
        std::set<vsomeip::eventgroup_t> eventgroups;
        eventgroups.insert(eventgroup_id_);
        app_->request_event(
            service_id_, instance_id_, event_id_, eventgroups,
            vsomeip::event_type_e::ET_EVENT);

        // Subscribe to the eventgroup
        app_->subscribe(service_id_, instance_id_, eventgroup_id_);

        // Register message handler for incoming events
        app_->register_message_handler(
            service_id_, instance_id_, event_id_,
            [this](const std::shared_ptr<vsomeip::message> &msg)
            {
              handle_message(msg);
            });

        // Register availability handler to track publisher count
        app_->register_availability_handler(
            service_id_, instance_id_,
            [this](vsomeip::service_t, vsomeip::instance_t, bool _is_available)
            {
              waitset_.set_publisher_count(_is_available ? 1 : 0);
            });

        waitset_.start();
      }
      catch (const std::exception &e)
      {
        cleanup();
        throw std::runtime_error("Failed to create vsomeip subscription: " + std::string(e.what()));
      }
    }

    ~Subscription()
    {
      cleanup();
    }

    Subscription(const Subscription &) = delete;
    Subscription &operator=(const Subscription &) = delete;
    Subscription(Subscription &&) = delete;
    Subscription &operator=(Subscription &&) = delete;

    int32_t get_publisher_count()
    {
      return waitset_.get_publisher_count();
    }

    void stop() override
    {
      waitset_.stop();
    }

    bool take(T &out_msg, lwrcl::MessageInfo &info)
    {
      return waitset_.take(out_msg, info);
    }

    bool has_message()
    {
      return waitset_.has_message();
    }

    bool has_data() { return has_message(); }

    size_t get_message_count() const
    {
      return 0;
    }

    std::string get_topic_name() const
    {
      return topic_name_;
    }

  private:
    void handle_message(const std::shared_ptr<vsomeip::message> &msg)
    {
      if (!msg || !msg->get_payload())
        return;

      auto payload = msg->get_payload();
      if (payload->get_length() == 0)
        return;

      try
      {
        // Deserialize the vsomeip payload using CycloneDDS CDR
        SerializedMessage serialized(payload->get_length());
        auto &raw = serialized.get_rcl_serialized_message();
        std::memcpy(raw.buffer, payload->get_data(), payload->get_length());
        raw.length = payload->get_length();

        auto message = std::make_shared<T>();
        Serialization<T>::deserialize_message(&serialized, message.get());

        // Push to waitset (which pushes to channel)
        waitset_.on_message_received(message);
      }
      catch (const std::exception &e)
      {
        std::cerr << "vsomeip deserialization error: " << e.what() << std::endl;
      }
    }

    void cleanup()
    {
      waitset_.stop();
      if (app_)
      {
        app_->unregister_message_handler(service_id_, instance_id_, event_id_);
        app_->unsubscribe(service_id_, instance_id_, eventgroup_id_);
        app_->release_event(service_id_, instance_id_, event_id_);
        app_->release_service(service_id_, instance_id_);
      }
    }

    std::shared_ptr<vsomeip::application> app_;
    SubscriberWaitSet<T> waitset_;
    std::string topic_name_;
    vsomeip::service_t service_id_;
    vsomeip::instance_t instance_id_;
    vsomeip::event_t event_id_;
    vsomeip::eventgroup_t eventgroup_id_;
  };

  class WaitSet
  {
  public:
    WaitSet() = default;

    template <typename T>
    WaitSet(std::initializer_list<std::shared_ptr<Subscription<T>>> init_list)
    {
      for (auto &sub : init_list)
      {
        add_subscription<T>(sub);
      }
    }

    template <typename T>
    void add_subscription(std::shared_ptr<Subscription<T>> sub)
    {
      SubscriptionHolder holder;
      holder.has_message = [sub]() -> bool
      { return sub->has_message(); };
      holder.take = [sub](void *data_ptr, MessageInfo &info) -> bool
      {
        T *typed_ptr = static_cast<T *>(data_ptr);
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
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
      return wait_for(ns);
    }

    WaitResult wait_for(std::chrono::nanoseconds timeout)
    {
      auto start = std::chrono::steady_clock::now();

      while (true)
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
