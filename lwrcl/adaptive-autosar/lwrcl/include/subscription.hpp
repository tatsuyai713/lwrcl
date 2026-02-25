#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
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
#include "lwrcl_autosar_proxy_skeleton.hpp"
#include "qos.hpp"

#define MAX_POLLABLE_BUFFER_SIZE 100

namespace lwrcl
{
  template <typename T>
  class LoanedSubscriptionMessage
  {
  public:
    struct SampleInfo
    {
      bool valid_data = false;
    };

    LoanedSubscriptionMessage() = default;

    explicit LoanedSubscriptionMessage(ara::com::SamplePtr<T> sample) noexcept
        : sample_(std::move(sample)),
          is_valid_(sample_ != nullptr)
    {
    }

    ~LoanedSubscriptionMessage() = default;

    LoanedSubscriptionMessage(const LoanedSubscriptionMessage &) = delete;
    LoanedSubscriptionMessage &operator=(const LoanedSubscriptionMessage &) = delete;

    LoanedSubscriptionMessage(LoanedSubscriptionMessage &&other) noexcept
        : sample_(std::move(other.sample_)),
          mutable_copy_(std::move(other.mutable_copy_)),
          is_valid_(other.is_valid_)
    {
      other.is_valid_ = false;
    }

    LoanedSubscriptionMessage &operator=(LoanedSubscriptionMessage &&other) noexcept
    {
      if (this != &other)
      {
        sample_ = std::move(other.sample_);
        mutable_copy_ = std::move(other.mutable_copy_);
        is_valid_ = other.is_valid_;
        other.is_valid_ = false;
      }
      return *this;
    }

    bool is_valid() const
    {
      return is_valid_ && sample_ != nullptr;
    }

    explicit operator bool() const
    {
      return is_valid();
    }

    T &get()
    {
      if (!is_valid())
      {
        throw std::runtime_error("Attempting to access invalid loaned message");
      }
      if (!mutable_copy_)
      {
        if (!sample_)
        {
          throw std::runtime_error("Attempting to access invalid loaned message");
        }
        mutable_copy_ = std::unique_ptr<T>(new T(*sample_));
      }
      return *mutable_copy_;
    }

    const T &get() const
    {
      if (!is_valid())
      {
        throw std::runtime_error("Attempting to access invalid loaned message");
      }
      if (mutable_copy_)
      {
        return *mutable_copy_;
      }
      return *sample_;
    }

    T *operator->()
    {
      return &get();
    }

    const T *operator->() const
    {
      return &get();
    }

    T &operator*()
    {
      return get();
    }

    const T &operator*() const
    {
      return get();
    }

    SampleInfo get_sample_info() const
    {
      return SampleInfo{is_valid()};
    }

    void release()
    {
      sample_.reset();
      mutable_copy_.reset();
      is_valid_ = false;
    }

  private:
    ara::com::SamplePtr<T> sample_;
    std::unique_ptr<T> mutable_copy_;
    bool is_valid_ = false;
  };

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
  class SubscriberWaitSet
  {
  public:
    SubscriberWaitSet(
        std::function<void(std::shared_ptr<T>)> callback_function,
        std::shared_ptr<std::mutex> node_mutex)
        : callback_function_(callback_function),
          node_mutex_(node_mutex),
          node_cv_(),
          node_cv_mutex_(),
          node_data_pending_(nullptr),
          proxy_(nullptr),
          stop_flag_(false),
          waitset_thread_(),
          proxy_mutex_(),
          data_cv_mutex_(),
          data_cv_(),
          data_pending_(false),
          lwrcl_subscriber_mutex_(std::make_shared<std::mutex>()),
          pollable_buffer_(),
          loaned_buffer_(),
          count_(0)
    {
    }

    ~SubscriberWaitSet() { stop(); }

    SubscriberWaitSet(const SubscriberWaitSet &) = delete;
    SubscriberWaitSet &operator=(const SubscriberWaitSet &) = delete;
    SubscriberWaitSet(SubscriberWaitSet &&) = delete;
    SubscriberWaitSet &operator=(SubscriberWaitSet &&) = delete;

    void ready(std::unique_ptr<autosar_generated::TopicEventProxy<T>> proxy)
    {
      proxy_ = std::move(proxy);
      stop_flag_.store(false);
    }

    void start()
    {
      if (proxy_) {
        // Register an event-driven receive handler to wake the run() thread
        proxy_->Event.SetReceiveHandler([this]() {
          {
            std::lock_guard<std::mutex> lk(data_cv_mutex_);
            data_pending_ = true;
          }
          data_cv_.notify_one();
        });
      }
      waitset_thread_ = std::thread(&SubscriberWaitSet::run, this);
    }

    void stop()
    {
      stop_flag_.store(true);
      data_cv_.notify_all();  // wake run() thread so it can exit
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

    // Register with the Node-level wakeup condition variable.
    // Stops the per-subscription thread so Node::spin() drives dispatch.
    void add_to_waitset(
        std::shared_ptr<std::condition_variable> cv,
        std::shared_ptr<std::mutex> cv_mutex,
        std::shared_ptr<std::atomic<bool>> pending)
    {
      node_cv_ = cv;
      node_cv_mutex_ = cv_mutex;
      node_data_pending_ = pending;
      stop(); // stop per-subscription thread
      // Re-register receive handler to signal the shared node cv.
      if (proxy_)
      {
        proxy_->Event.SetReceiveHandler([this]() {
          if (node_data_pending_) node_data_pending_->store(true);
          if (node_cv_) node_cv_->notify_all();
        });
      }
    }

    // Invoke callback for any available data (called from Node::spin() after wakeup).
    void invoke_if_data()
    {
      try { run_once(); }
      catch (const std::exception &e) {
        std::cerr << "Exception in invoke_if_data: " << e.what() << std::endl;
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

    bool take_loaned(LoanedSubscriptionMessage<T> &out_loaned)
    {
      {
        std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
        if (!loaned_buffer_.empty())
        {
          auto sample = std::move(loaned_buffer_.front());
          loaned_buffer_.pop_front();
          out_loaned = LoanedSubscriptionMessage<T>(std::move(sample));
          return out_loaned.is_valid();
        }
      }

      std::lock_guard<std::mutex> proxy_lock(proxy_mutex_);
      if (!proxy_)
      {
        return false;
      }

      const auto state = proxy_->Event.GetSubscriptionState();
      count_.store(state == ara::com::SubscriptionState::kSubscribed ? 1 : 0);

      bool taken = false;
      auto result = proxy_->Event.GetNewSamples(
          [&out_loaned, &taken](ara::com::SamplePtr<T> payload_sample)
          {
            if (taken || !payload_sample)
            {
              return;
            }
            out_loaned = LoanedSubscriptionMessage<T>(std::move(payload_sample));
            taken = out_loaned.is_valid();
          },
          1);

      return result.HasValue() && taken;
    }

  private:
    void push_message(std::shared_ptr<T> message, ara::com::SamplePtr<T> loaned_sample)
    {
      lwrcl::MessageInfo new_info;
      new_info.source_timestamp = std::chrono::system_clock::now();
      new_info.from_intra_process = false;

      {
        std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
        pollable_buffer_.emplace_back(message, new_info);
        if (loaned_sample)
          loaned_buffer_.emplace_back(std::move(loaned_sample));
        if (pollable_buffer_.size() > MAX_POLLABLE_BUFFER_SIZE)
        {
          pollable_buffer_.pop_front();
          if (!loaned_buffer_.empty()) loaned_buffer_.pop_front();
        }
      }

      // Direct invocation — no channel/executor hop.
      {
        std::lock_guard<std::mutex> lock(*node_mutex_);
        try { callback_function_(message); }
        catch (const std::exception &e) {
          std::cerr << "Exception in subscription callback: " << e.what() << std::endl;
        } catch (...) {
          std::cerr << "Unknown exception in subscription callback." << std::endl;
        }
      }
    }

    void run_once()
    {
      if (!proxy_)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return;
      }

      std::lock_guard<std::mutex> proxy_lock(proxy_mutex_);
      if (!proxy_)
      {
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
            push_message(message, std::move(payload_sample));
          },
          10);
    }

    void run()
    {
      while (!stop_flag_.load())
      {
        // Wait for SetReceiveHandler to signal new data (or 100 ms for state polling)
        {
          std::unique_lock<std::mutex> lk(data_cv_mutex_);
          data_cv_.wait_for(lk, std::chrono::milliseconds(100),
                            [this]{ return data_pending_ || stop_flag_.load(); });
          data_pending_ = false;
        }

        if (stop_flag_.load()) break;

        try
        {
          run_once();
        }
        catch (const std::exception &e)
        {
          std::cerr << "Exception in SubscriberWaitSet: " << e.what() << std::endl;
        }
      }
    }

    std::function<void(std::shared_ptr<T>)> callback_function_;
    std::shared_ptr<std::mutex> node_mutex_;
    std::shared_ptr<std::condition_variable> node_cv_;
    std::shared_ptr<std::mutex> node_cv_mutex_;
    std::shared_ptr<std::atomic<bool>> node_data_pending_;
    std::unique_ptr<autosar_generated::TopicEventProxy<T>> proxy_;
    std::atomic<bool> stop_flag_;
    std::thread waitset_thread_;
    std::mutex proxy_mutex_;
    std::mutex data_cv_mutex_;
    std::condition_variable data_cv_;
    bool data_pending_{false};

    std::shared_ptr<std::mutex> lwrcl_subscriber_mutex_;
    std::deque<std::pair<std::shared_ptr<T>, lwrcl::MessageInfo>> pollable_buffer_;
    std::deque<ara::com::SamplePtr<T>> loaned_buffer_;
    std::atomic<int32_t> count_{0};
  };

  class ISubscription
  {
  public:
    virtual ~ISubscription() = default;
    virtual void stop() = 0;
    virtual void add_to_waitset(
        std::shared_ptr<std::condition_variable> cv,
        std::shared_ptr<std::mutex> cv_mutex,
        std::shared_ptr<std::atomic<bool>> pending) = 0;
    virtual void invoke_if_data() = 0;

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
        std::shared_ptr<std::mutex> node_mutex)
        : waitset_(callback_function, node_mutex),
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

    void add_to_waitset(
        std::shared_ptr<std::condition_variable> cv,
        std::shared_ptr<std::mutex> cv_mutex,
        std::shared_ptr<std::atomic<bool>> pending) override
    {
      waitset_.add_to_waitset(cv, cv_mutex, pending);
    }

    void invoke_if_data() override
    {
      waitset_.invoke_if_data();
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

    bool take_loaned_message(LoanedSubscriptionMessage<T> &out_loaned)
    {
      return waitset_.take_loaned(out_loaned);
    }

    // Loaned API is always available; runtime binding decides copy/zero-copy path.
    bool can_loan_messages() const
    {
      return true;
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
      auto proxy = std::make_unique<autosar_generated::TopicEventProxy<T>>(topic_name_);
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
