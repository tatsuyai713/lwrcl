#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <deque>
#include <vector>

#include <dds/dds.hpp>
#include "qos.hpp"
#include "channel.hpp"

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
          reader_(nullptr),
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
    SubscriberWaitSet(SubscriberWaitSet &&) = default;
    SubscriberWaitSet &operator=(SubscriberWaitSet &&) = default;

    void ready(std::shared_ptr<dds::sub::DataReader<T>> reader)
    {
      reader_ = reader;
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
    void run()
    {
      lwrcl::MessageInfo new_info;

      while (!stop_flag_.load())
      {
        try {
          if (!reader_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }

          // Check for subscription matches
          try {
            auto matched_status = reader_->subscription_matched_status();
            count_.store(matched_status.current_count());
          } catch (const dds::core::Exception& e) {
            // Status not available, continue
          }

          // Try to read available data
          dds::sub::LoanedSamples<T> samples = reader_->take();
          
          if (samples.length() > 0) {
            // Keep the loan alive until callbacks and pollable buffer references are done.
            auto loan_guard = std::make_shared<dds::sub::LoanedSamples<T>>(std::move(samples));
            for (auto &sample : *loan_guard) {
              if (sample.info().valid()) {
                // Alias shared_ptr to the loaned sample to avoid extra copy.
                auto message_view =
                    std::shared_ptr<T>(loan_guard, const_cast<T *>(&sample.data()));

                channel_->produce(std::make_shared<SubscriberCallback<T>>(
                    callback_function_, message_view, lwrcl_subscriber_mutex_));

                new_info.source_timestamp = std::chrono::system_clock::now();
                new_info.from_intra_process = false;
                pollable_buffer_.emplace_back(message_view, new_info);
                
                if (pollable_buffer_.size() > MAX_POLLABLE_BUFFER_SIZE) {
                  pollable_buffer_.pop_front();
                }
              }
            }
          }
        }
        catch (const dds::core::Exception& e) {
          std::cerr << "DDS Exception in SubscriberWaitSet: " << e.what() << std::endl;
        }
        catch (const std::exception& e) {
          std::cerr << "Exception in SubscriberWaitSet: " << e.what() << std::endl;
        }

        // Small sleep to prevent busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    std::function<void(std::shared_ptr<T>)> callback_function_;
    CallbackChannel::SharedPtr channel_;
    std::shared_ptr<dds::sub::DataReader<T>> reader_;
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
        dds::domain::DomainParticipant *participant, const std::string &topic_name,
        const QoS &qos, std::function<void(std::shared_ptr<T>)> callback_function,
        CallbackChannel::SharedPtr channel)
        : participant_(participant),
          waitset_(callback_function, channel),
          topic_(nullptr),
          subscriber_(nullptr),
          reader_(nullptr),
          topic_owned_(false)
    {
      try {
        // Create subscriber
        subscriber_ = std::make_shared<dds::sub::Subscriber>(*participant_);
        
        // Create topic
        dds::topic::qos::TopicQos topic_qos;
        
        // Create topic directly (Cyclone DDS doesn't require topic reuse checking)
        topic_ = std::make_shared<dds::topic::Topic<T>>(*participant_, topic_name, topic_qos);
        topic_owned_ = true;

        // Configure DataReader QoS
        dds::sub::qos::DataReaderQos reader_qos;
        
        // Set history policy
        if (qos.get_history() == QoS::HistoryPolicy::KEEP_ALL) {
          reader_qos << dds::core::policy::History::KeepAll();
        } else {
          reader_qos << dds::core::policy::History::KeepLast(qos.get_depth());
        }
        
        // Set reliability policy
        if (qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT) {
          reader_qos << dds::core::policy::Reliability::BestEffort();
        } else {
          reader_qos << dds::core::policy::Reliability::Reliable();
        }
        
        // Set durability policy
        if (qos.get_durability() == QoS::DurabilityPolicy::VOLATILE) {
          reader_qos << dds::core::policy::Durability::Volatile();
        } else {
          reader_qos << dds::core::policy::Durability::TransientLocal();
        }

        // Create DataReader without listener (listener handling will be done differently)
        reader_ = std::make_shared<dds::sub::DataReader<T>>(*subscriber_, *topic_, reader_qos);
        
        // Initialize and start the waitset
        waitset_.ready(reader_);
        waitset_.start();
        
      } catch (const dds::core::Exception& e) {
        cleanup();
        throw std::runtime_error("Failed to create subscription: " + std::string(e.what()));
      }
    }

    ~Subscription()
    {
      cleanup();
    }

    Subscription(const Subscription &) = delete;
    Subscription &operator=(const Subscription &) = delete;
    Subscription(Subscription &&) = default;
    Subscription &operator=(Subscription &&) = default;

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

    int32_t get_publisher_count() const
    {
      if (!reader_) {
        return 0;
      }
      
      try {
        auto matched_status = reader_->subscription_matched_status();
        return matched_status.current_count();
      } catch (const dds::core::Exception& e) {
        return 0;
      }
    }

  private:
    void cleanup()
    {
      waitset_.stop();
      
      // Cyclone DDS uses RAII, so explicit cleanup is generally not needed
      // However, we can reset shared_ptrs to ensure proper destruction order
      if (reader_) {
        reader_.reset();
      }
      if (subscriber_) {
        subscriber_.reset();
      }
      if (topic_ && topic_owned_) {
        topic_.reset();
      }
    }

    dds::domain::DomainParticipant *participant_;
    SubscriberWaitSet<T> waitset_;
    std::shared_ptr<dds::topic::Topic<T>> topic_;
    std::shared_ptr<dds::sub::Subscriber> subscriber_;
    std::shared_ptr<dds::sub::DataReader<T>> reader_;
    bool topic_owned_;
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
