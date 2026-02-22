#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <chrono>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "adaptive_autosar_header.hpp"
#include "channel.hpp"
#include "lwrcl_autosar_proxy_skeleton.hpp"
#include "qos.hpp"

#define MAX_POLLABLE_BUFFER_SIZE 100

namespace lwrcl
{

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

  template <typename T>
  class SubscriberWaitSet
  {
  public:
    SubscriberWaitSet(
        std::function<void(std::shared_ptr<T>)> callback_function,
        CallbackChannel::SharedPtr channel)
        : callback_function_(callback_function),
          channel_(channel),
          proxy_(nullptr),
          stop_flag_(false),
          waitset_thread_(),
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

    void ready(std::unique_ptr<lwrcl::autosar_generated::TopicEventProxy<T>> proxy)
    {
      proxy_ = std::move(proxy);
      stop_flag_.store(false);
    }

    void start()
    {
      waitset_thread_ = std::thread(&SubscriberWaitSet::run, this);
    }

    void stop()
    {
      stop_flag_.store(true);
      if (waitset_thread_.joinable())
      {
        waitset_thread_.join();
      }
      if (proxy_)
      {
        proxy_->Event.UnsetReceiveHandler();
        proxy_->Event.Unsubscribe();
      }
    }

    int32_t get_publisher_count()
    {
      return count_.load();
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
    void push_message(std::shared_ptr<T> message)
    {
      channel_->produce(std::make_shared<SubscriberCallback<T>>(
          callback_function_, message, lwrcl_subscriber_mutex_));

      lwrcl::MessageInfo new_info;
      new_info.source_timestamp = std::chrono::system_clock::now();
      new_info.from_intra_process = false;

      std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
      pollable_buffer_.emplace_back(message, new_info);

      if (pollable_buffer_.size() > MAX_POLLABLE_BUFFER_SIZE)
      {
        pollable_buffer_.pop_front();
      }
    }

    void run_once()
    {
      if (!proxy_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return;
      }

      const auto state = proxy_->Event.GetSubscriptionState();
      count_.store(state == ara::com::SubscriptionState::kSubscribed ? 1 : 0);
      (void)proxy_->Event.GetNewSamples(
          [this](ara::com::SamplePtr<T> payload_sample)
          {
            if (!payload_sample)
            {
              return;
            }

            auto message = std::make_shared<T>(*payload_sample);
            push_message(message);
          },
          10);
    }

    void run()
    {
      while (!stop_flag_.load())
      {
        try
        {
          run_once();
        }
        catch (const std::exception &e)
        {
          std::cerr << "Exception in SubscriberWaitSet: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    std::function<void(std::shared_ptr<T>)> callback_function_;
    CallbackChannel::SharedPtr channel_;
    std::unique_ptr<lwrcl::autosar_generated::TopicEventProxy<T>> proxy_;
    std::atomic<bool> stop_flag_;
    std::thread waitset_thread_;

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
        AutosarDomainParticipant *participant, const std::string &topic_name,
        const QoS &qos, std::function<void(std::shared_ptr<T>)> callback_function,
        CallbackChannel::SharedPtr channel)
        : waitset_(callback_function, channel),
          topic_name_(topic_name)
    {
      (void)participant;
      (void)qos; // QoS is fixed in Adaptive AUTOSAR DDS layer

      try
      {
        initialize_adapter();

        waitset_.start();
      }
      catch (const std::exception &e)
      {
        throw std::runtime_error("Failed to create subscription: " + std::string(e.what()));
      }
    }

    ~Subscription()
    {
      waitset_.stop();
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

    // Alias for rclcpp compatibility
    bool has_data() { return has_message(); }

    // No zero-copy support in Adaptive AUTOSAR backend
    bool can_loan_messages() const
    {
      return false;
    }

    // Get the number of messages waiting (approximation)
    size_t get_message_count() const
    {
      return 0;
    }

    std::string get_topic_name() const
    {
      return topic_name_;
    }

  private:
    void initialize_adapter()
    {
      auto proxy = std::make_unique<lwrcl::autosar_generated::TopicEventProxy<T>>(topic_name_);
      proxy->Event.Subscribe(MAX_POLLABLE_BUFFER_SIZE);
      waitset_.ready(std::move(proxy));
    }

    SubscriberWaitSet<T> waitset_;
    std::string topic_name_;
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
