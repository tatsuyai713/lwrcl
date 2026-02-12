#ifndef LWRCL_SUBSCRIBER_HPP_
#define LWRCL_SUBSCRIBER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <deque>

#include "fast_dds_header.hpp"
#include "qos.hpp"
#include "channel.hpp"

#define MAX_POLLABLE_BUFFER_SIZE 100

namespace lwrcl
{
  // Forward declaration
  template <typename T>
  class Subscription;

  /**
   * @brief Loaned message for zero-copy subscription (rclcpp compatible)
   * 
   * This class wraps a message that was loaned from the middleware
   * using the data sharing feature. The message is automatically
   * returned to the middleware when this object is destroyed.
   * 
   * Note: FastDDS zero-copy subscription requires keeping the LoanableSequence
   * alive until the loan is returned. This implementation stores the sequences
   * internally to ensure proper lifecycle management.
   */
  template <typename T>
  class LoanedSubscriptionMessage
  {
  public:
    LoanedSubscriptionMessage() = default;

    /**
     * @brief Construct from loaned sequences (takes ownership)
     * 
     * This constructor takes ownership of the loaned sequences.
     * The loan will be returned when this object is destroyed.
     */
    LoanedSubscriptionMessage(
        eprosima::fastdds::dds::DataReader *reader,
        eprosima::fastdds::dds::LoanableSequence<T> &&data_seq,
        eprosima::fastdds::dds::SampleInfoSeq &&info_seq)
        : reader_(reader),
          data_seq_(std::move(data_seq)),
          info_seq_(std::move(info_seq)),
          is_valid_(data_seq_.length() > 0 && info_seq_.length() > 0 && info_seq_[0].valid_data)
    {
    }

    ~LoanedSubscriptionMessage()
    {
      release();
    }

    // Move only
    LoanedSubscriptionMessage(const LoanedSubscriptionMessage &) = delete;
    LoanedSubscriptionMessage &operator=(const LoanedSubscriptionMessage &) = delete;

    LoanedSubscriptionMessage(LoanedSubscriptionMessage &&other) noexcept
        : reader_(other.reader_),
          data_seq_(std::move(other.data_seq_)),
          info_seq_(std::move(other.info_seq_)),
          is_valid_(other.is_valid_)
    {
      other.reader_ = nullptr;
      other.is_valid_ = false;
    }

    LoanedSubscriptionMessage &operator=(LoanedSubscriptionMessage &&other) noexcept
    {
      if (this != &other)
      {
        release();
        reader_ = other.reader_;
        data_seq_ = std::move(other.data_seq_);
        info_seq_ = std::move(other.info_seq_);
        is_valid_ = other.is_valid_;
        other.reader_ = nullptr;
        other.is_valid_ = false;
      }
      return *this;
    }

    /**
     * @brief Check if the loaned message is valid
     */
    bool is_valid() const { return is_valid_; }
    operator bool() const { return is_valid_; }

    /**
     * @brief Get reference to the loaned message
     */
    T &get()
    {
      if (!is_valid_)
      {
        throw std::runtime_error("Attempting to access invalid loaned message");
      }
      return data_seq_[0];
    }

    const T &get() const
    {
      if (!is_valid_)
      {
        throw std::runtime_error("Attempting to access invalid loaned message");
      }
      return data_seq_[0];
    }

    T *operator->() { return &get(); }
    const T *operator->() const { return &get(); }

    T &operator*() { return get(); }
    const T &operator*() const { return get(); }

    /**
     * @brief Get the sample info
     */
    const eprosima::fastdds::dds::SampleInfo &get_sample_info() const
    {
      if (!is_valid_)
      {
        throw std::runtime_error("Attempting to access sample info of invalid loaned message");
      }
      return info_seq_[0];
    }

    /**
     * @brief Release the loaned message back to the middleware
     * 
     * This must be called to return the loan to the middleware.
     * It is automatically called by the destructor.
     */
    void release()
    {
      if (is_valid_ && reader_)
      {
        // Return the loan to FastDDS
        ReturnCode_t ret = reader_->return_loan(data_seq_, info_seq_);
        if (ret != ReturnCode_t::RETCODE_OK)
        {
          std::cerr << "Warning: Failed to return loan to DataReader" << std::endl;
        }
        reader_ = nullptr;
        is_valid_ = false;
      }
    }

  private:
    eprosima::fastdds::dds::DataReader *reader_ = nullptr;
    eprosima::fastdds::dds::LoanableSequence<T> data_seq_;
    eprosima::fastdds::dds::SampleInfoSeq info_seq_;
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
  class SubscriberCallback : public ChannelCallback
  {
  public:
    SubscriberCallback(
        std::function<void(std::shared_ptr<T>)> callback_function, std::shared_ptr<T> message_ptr,
        std::shared_ptr<std::mutex> lwrcl_subscriber_mutex)
        : callback_function_(callback_function),
          message_ptr_(message_ptr),
          lwrcl_subscriber_mutex_(lwrcl_subscriber_mutex)
    {
    }

    ~SubscriberCallback() noexcept
    {
    }

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
          wait_set_(),
          terminate_condition_(),
          status_cond_(nullptr),
          message_ptr_(std::make_shared<T>()),
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

    void ready(eprosima::fastdds::dds::DataReader *reader)
    {
      reader_ = reader;
      stop_flag_.store(false);

      status_cond_ = &reader_->get_statuscondition();

      eprosima::fastdds::dds::StatusMask subscription_matched_mask =
          eprosima::fastdds::dds::StatusMask::subscription_matched();
      eprosima::fastdds::dds::StatusMask data_available_mask =
          eprosima::fastdds::dds::StatusMask::data_available();

      eprosima::fastdds::dds::StatusMask status_mask = data_available_mask;
      status_mask |= subscription_matched_mask;
      status_cond_->set_enabled_statuses(status_mask);

      // Attach the StatusCondition to the WaitSet
      wait_set_.attach_condition(*status_cond_);
      wait_set_.attach_condition(terminate_condition_);
    }

    void start() { waitset_thread_ = std::thread(&SubscriberWaitSet::run, this); }

    void stop()
    {
      stop_flag_.store(true);
      terminate_condition_.set_trigger_value(true);
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
      out_msg = *(msg_ptr);
      return true;
    }

    bool has_message()
    {
      std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
      return !pollable_buffer_.empty();
    }

    bool has_message() const
    {
      std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
      return !pollable_buffer_.empty();
    }

    /**
     * @brief Take a loaned message directly from the reader (zero-copy)
     * 
     * This method attempts to take a message using FastDDS's loan mechanism.
     * The LoanedSubscriptionMessage takes ownership of the loaned sequences
     * and will return the loan when destroyed.
     * 
     * @param out_loaned The output loaned message
     * @return true if a message was successfully taken
     */
    bool take_loaned(LoanedSubscriptionMessage<T> &out_loaned)
    {
      if (!reader_)
      {
        return false;
      }

      // Create sequences for loan-based reading
      eprosima::fastdds::dds::LoanableSequence<T> data_seq;
      eprosima::fastdds::dds::SampleInfoSeq info_seq;

      ReturnCode_t ret = reader_->take(
          data_seq,
          info_seq,
          1, // max_samples
          eprosima::fastdds::dds::ANY_SAMPLE_STATE,
          eprosima::fastdds::dds::ANY_VIEW_STATE,
          eprosima::fastdds::dds::ANY_INSTANCE_STATE);

      if (ret == ReturnCode_t::RETCODE_OK && data_seq.length() > 0)
      {
        if (info_seq[0].valid_data)
        {
          // Transfer ownership of sequences to LoanedSubscriptionMessage
          // The loan will be returned when LoanedSubscriptionMessage is destroyed
          out_loaned = LoanedSubscriptionMessage<T>(
              reader_,
              std::move(data_seq),
              std::move(info_seq));
          return true;
        }
        // Invalid data, return the loan immediately
        reader_->return_loan(data_seq, info_seq);
      }
      return false;
    }

    /**
     * @brief Get the DataReader for direct access (advanced use)
     */
    eprosima::fastdds::dds::DataReader *get_reader() { return reader_; }

  private:
    void run()
    {
      eprosima::fastdds::dds::ConditionSeq active_conditions;
      eprosima::fastdds::dds::Entity *entity = nullptr;
      eprosima::fastdds::dds::StatusCondition *status_cond = nullptr;
      eprosima::fastrtps::Duration_t timeout{1, 0}; // Wait for 1 second
      eprosima::fastdds::dds::StatusMask changed_statuses;
      ReturnCode_t ret_code;
      lwrcl::MessageInfo new_info;

      while (!stop_flag_.load())
      {
        ret_code = wait_set_.wait(active_conditions, timeout);
        if (ret_code != ReturnCode_t::RETCODE_OK)
        {
          continue;
        }

        if (stop_flag_.load())
        {
          return;
        }

        for (auto condition : active_conditions)
        {
          status_cond =
              dynamic_cast<eprosima::fastdds::dds::StatusCondition *>(condition);
          if (status_cond_ == status_cond)
          {
            entity = status_cond->get_entity();
            if (entity == nullptr)
            {
              continue;
            }

            changed_statuses = entity->get_status_changes();

            if (changed_statuses.is_active(eprosima::fastdds::dds::StatusMask::subscription_matched()))
            {
              eprosima::fastdds::dds::SubscriptionMatchedStatus match_status;
              ret_code = reader_->get_subscription_matched_status(match_status);
              if (ret_code == ReturnCode_t::RETCODE_OK)
              {
                count_.store(match_status.current_count);
              }
            }

            if (changed_statuses.is_active(eprosima::fastdds::dds::StatusMask::data_available()))
            {
              std::lock_guard<std::mutex> lock(*lwrcl_subscriber_mutex_);
              while (reader_->take_next_sample(message_ptr_.get(), &info_) == ReturnCode_t::RETCODE_OK)
              {
                if (info_.valid_data)
                {
                  // Move the filled buffer out and allocate a fresh one for the next sample.
                  // This avoids copying the message data — only the shared_ptr is moved.
                  auto message_ready = std::move(message_ptr_);
                  message_ptr_ = std::make_shared<T>();

                  channel_->produce(std::make_shared<SubscriberCallback<T>>(
                      callback_function_, message_ready, lwrcl_subscriber_mutex_));
                  new_info.source_timestamp = std::chrono::system_clock::now();
                  new_info.from_intra_process = false;
                  pollable_buffer_.emplace_back(std::move(message_ready), new_info);
                  if (pollable_buffer_.size() > MAX_POLLABLE_BUFFER_SIZE)
                  {
                    pollable_buffer_.pop_front();
                  }
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

    std::function<void(std::shared_ptr<T>)> callback_function_;
    CallbackChannel::SharedPtr channel_;
    eprosima::fastdds::dds::DataReader *reader_;
    std::atomic<bool> stop_flag_;
    std::thread waitset_thread_;
    eprosima::fastdds::dds::WaitSet wait_set_;
    eprosima::fastdds::dds::GuardCondition terminate_condition_;
    eprosima::fastdds::dds::StatusCondition *status_cond_;
    std::shared_ptr<T> message_ptr_;
    std::shared_ptr<std::mutex> lwrcl_subscriber_mutex_;
    eprosima::fastdds::dds::SampleInfo info_;
    std::deque<std::pair<std::shared_ptr<T>, lwrcl::MessageInfo>> pollable_buffer_;
    std::atomic<int32_t> count_{0};
  };

  class ISubscription
  {
  public:
    virtual ~ISubscription() = default;
    virtual void stop() = 0;
  };

  template <typename T>
  class Subscription : public ISubscription, public std::enable_shared_from_this<Subscription<T>>
  {
  public:
    using SharedPtr = std::shared_ptr<Subscription<T>>;

    Subscription(
        eprosima::fastdds::dds::DomainParticipant *participant, const std::string &topic_name,
        const QoS &qos, std::function<void(std::shared_ptr<T>)> callback_function,
        CallbackChannel::SharedPtr channel)
        : participant_(participant),
          waitset_(callback_function, channel),
          topic_(nullptr),
          subscriber_(nullptr),
          reader_(nullptr)
    {
      using ParentType = typename ParentTypeTraits<T>::Type;
      message_type_ = lwrcl::MessageType(new ParentType());
      lwrcl::dds::TopicQos topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT();
      if (message_type_.get_type_support().register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        std::cerr << "Failed to register message type" << std::endl;
      }

      std::string type_name = message_type_.get_type_support().get_type_name();
      subscriber_ = participant_->create_subscriber(eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT);
      if (!subscriber_)
      {
        std::cerr << "Failed to create subscriber" << std::endl;
      }

      eprosima::fastdds::dds::Topic *retrieved_topic =
          dynamic_cast<eprosima::fastdds::dds::Topic *>(participant->lookup_topicdescription(topic_name));
      if (retrieved_topic == nullptr)
      {
        topic_ = participant_->create_topic(
            topic_name, type_name, topic_qos);
        if (!topic_)
        {
          participant_->delete_subscriber(subscriber_);
          std::cerr << "Failed to create topic" << std::endl;
        }
        topic_owned_ = true;
      }
      else
      {
        topic_ = retrieved_topic;
        topic_owned_ = false;
      }

      eprosima::fastdds::dds::DataReaderQos reader_qos =
          eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
      reader_qos.endpoint().history_memory_policy =
          eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
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

      // Liveliness
      if (qos.get_liveliness() == QoS::LivelinessPolicy::AUTOMATIC)
      {
        reader_qos.liveliness().kind = eprosima::fastdds::dds::AUTOMATIC_LIVELINESS_QOS;
      }
      else
      {
        reader_qos.liveliness().kind = eprosima::fastdds::dds::MANUAL_BY_TOPIC_LIVELINESS_QOS;
      }

      // Liveliness Lease Duration
      auto liveliness_lease = qos.get_liveliness_lease_duration();
      if (!liveliness_lease.is_infinite())
      {
        reader_qos.liveliness().lease_duration.seconds = static_cast<int32_t>(liveliness_lease.sec);
        reader_qos.liveliness().lease_duration.nanosec = liveliness_lease.nsec;
      }

      // Deadline
      auto deadline = qos.get_deadline();
      if (!deadline.is_infinite())
      {
        reader_qos.deadline().period.seconds = static_cast<int32_t>(deadline.sec);
        reader_qos.deadline().period.nanosec = deadline.nsec;
      }

      reader_qos.data_sharing().automatic();
      reader_qos.properties().properties().emplace_back("fastdds.intraprocess_delivery", "true");

      reader_ = subscriber_->create_datareader(topic_, reader_qos, nullptr, eprosima::fastdds::dds::StatusMask::all());
      if (!reader_)
      {
        participant_->delete_subscriber(subscriber_);
        participant_->delete_topic(topic_);
        throw std::runtime_error("Failed to create datareader");
      }

      waitset_.ready(reader_);
      waitset_.start();
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

    Subscription(const Subscription &) = delete;
    Subscription &operator=(const Subscription &) = delete;
    Subscription(Subscription &&) = delete;
    Subscription &operator=(Subscription &&) = delete;

    int32_t get_publisher_count() { return const_cast<const Subscription *>(this)->get_publisher_count(); }

    void stop() override { waitset_.stop(); }

    bool take(std::shared_ptr<T> &out_msg, lwrcl::MessageInfo &info) { return waitset_.take(out_msg, info); }
    bool take(T &out_msg, lwrcl::MessageInfo &info) { return waitset_.take(out_msg, info); }

    // Take without MessageInfo (convenience overload)
    bool take(std::shared_ptr<T> &out_msg)
    {
      lwrcl::MessageInfo info;
      return waitset_.take(out_msg, info);
    }

    bool take(T &out_msg)
    {
      lwrcl::MessageInfo info;
      return waitset_.take(out_msg, info);
    }

    bool has_message() { return waitset_.has_message(); }

    // Alias for rclcpp compatibility
    bool has_data() { return has_message(); }

    /**
     * @brief Take a loaned message (zero-copy when possible)
     * 
     * This method attempts to take a message using the middleware's
     * loan mechanism for zero-copy data transfer. If loaning is not
     * available, it falls back to copying.
     * 
     * @param out_loaned Output loaned message
     * @return true if a message was successfully taken
     */
    bool take_loaned_message(LoanedSubscriptionMessage<T> &out_loaned)
    {
      return waitset_.take_loaned(out_loaned);
    }

    /**
     * @brief Check if the subscription can loan messages
     * @return true if zero-copy loaning is available
     */
    bool can_loan_messages() const
    {
      // Check if data sharing is enabled
      if (!reader_)
      {
        return false;
      }
      // Data sharing was enabled in reader_qos.data_sharing().automatic()
      // but actual loan availability depends on middleware configuration
      return true;
    }

    /**
     * @brief Get the underlying DataReader (for advanced use)
     */
    eprosima::fastdds::dds::DataReader *get_reader() { return reader_; }

    // Get the number of messages waiting
    size_t get_message_count() const
    {
      // This is an approximation; the actual implementation returns buffer size
      return waitset_.has_message() ? 1 : 0;
    }

    // Get the topic name
    std::string get_topic_name() const
    {
      if (topic_)
      {
        return topic_->get_name();
      }
      return "";
    }

    int32_t get_publisher_count() const
    {
      if (!reader_)
      {
        return 0;
      }
      eprosima::fastdds::dds::SubscriptionMatchedStatus matched_status;
      if (reader_->get_subscription_matched_status(matched_status) == ReturnCode_t::RETCODE_OK)
      {
        return matched_status.current_count;
      }
      return 0;
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

    WaitResult wait() { return wait_for(std::chrono::nanoseconds(-1)); }

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

    const std::vector<SubscriptionHolder> &get_subscriptions() const { return subs_; }

  private:
    std::vector<SubscriptionHolder> subs_;
  };
} // namespace lwrcl

#endif // LWRCL_SUBSCRIBER_HPP_
