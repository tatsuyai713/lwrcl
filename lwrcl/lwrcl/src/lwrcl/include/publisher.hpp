#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "fast_dds_header.hpp"
#include "qos.hpp"

namespace lwrcl
{

  // Forward declaration for LoanedMessage
  template <typename T>
  class LoanedMessage;

  class PublisherListener : public dds::DataWriterListener
  {
  public:
    void on_publication_matched(dds::DataWriter *, const dds::PublicationMatchedStatus &status) override
    {
      count = status.current_count;
    }

    std::atomic<int32_t> count{0};
  };

  class IPublisher
  {
  public:
    virtual ~IPublisher() = default;

    IPublisher(const IPublisher &) = delete;
    IPublisher &operator=(const IPublisher &) = delete;
    IPublisher(IPublisher &&) = default;
    IPublisher &operator=(IPublisher &&) = default;
    virtual int32_t get_subscriber_count() = 0;

  protected:
    IPublisher() = default;
  };

  template <typename T>
  class Publisher : public IPublisher, public std::enable_shared_from_this<Publisher<T>>
  {
  public:
    Publisher(dds::DomainParticipant *participant, const std::string &topic_name, const QoS &qos)
        : IPublisher(),
          std::enable_shared_from_this<Publisher<T>>(),
          participant_(participant),
          topic_(nullptr),
          publisher_(nullptr),
          writer_(nullptr)
    {
      lwrcl::dds::TopicQos topic_qos = lwrcl::dds::TOPIC_QOS_DEFAULT();

      using ParentType = typename ParentTypeTraits<T>::Type;
      message_type_ = lwrcl::MessageType(new ParentType());

      if (message_type_.get_type_support().register_type(participant_) != ReturnCode_t::RETCODE_OK)
      {
        throw std::runtime_error("Failed to register message type");
      }

      std::string type_name = message_type_.get_type_support().get_type_name();

      publisher_ = participant_->create_publisher(dds::PUBLISHER_QOS_DEFAULT());
      if (!publisher_)
      {
        throw std::runtime_error("Failed to create publisher");
      }
      dds::Topic *retrieved_topic = dynamic_cast<eprosima::fastdds::dds::Topic *>(participant->lookup_topicdescription(topic_name));
      if (retrieved_topic == nullptr)
      {
        topic_ = participant_->create_topic(topic_name, type_name, topic_qos);
        if (!topic_)
        {
          participant_->delete_publisher(publisher_); // Cleanup on failure
          throw std::runtime_error("Failed to create topic");
        }
        topic_owned_ = true;
      }
      else
      {
        topic_ = retrieved_topic;
        topic_owned_ = false;
      }

      dds::DataWriterQos writer_qos = dds::DATAWRITER_QOS_DEFAULT();
      writer_qos.endpoint().history_memory_policy = rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

      writer_qos.history().depth = qos.get_depth();
      if (qos.get_history() == QoS::HistoryPolicy::KEEP_ALL)
      {
        writer_qos.history().kind = eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
      }
      else
      {
        writer_qos.history().kind = eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
      }
      if (qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT)
      {
        writer_qos.reliability().kind = eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
      }
      else
      {
        writer_qos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
      }
      if (qos.get_durability() == QoS::DurabilityPolicy::VOLATILE)
      {
        writer_qos.durability().kind = eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
      }
      else
      {
        writer_qos.durability().kind = eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
      }

      // Liveliness
      if (qos.get_liveliness() == QoS::LivelinessPolicy::AUTOMATIC)
      {
        writer_qos.liveliness().kind = eprosima::fastdds::dds::AUTOMATIC_LIVELINESS_QOS;
      }
      else
      {
        writer_qos.liveliness().kind = eprosima::fastdds::dds::MANUAL_BY_TOPIC_LIVELINESS_QOS;
      }

      // Liveliness Lease Duration
      auto liveliness_lease = qos.get_liveliness_lease_duration();
      if (!liveliness_lease.is_infinite())
      {
        writer_qos.liveliness().lease_duration.seconds = static_cast<int32_t>(liveliness_lease.sec);
        writer_qos.liveliness().lease_duration.nanosec = liveliness_lease.nsec;
      }

      // Deadline
      auto deadline = qos.get_deadline();
      if (!deadline.is_infinite())
      {
        writer_qos.deadline().period.seconds = static_cast<int32_t>(deadline.sec);
        writer_qos.deadline().period.nanosec = deadline.nsec;
      }

      // Lifespan
      auto lifespan = qos.get_lifespan();
      if (!lifespan.is_infinite())
      {
        writer_qos.lifespan().duration.seconds = static_cast<int32_t>(lifespan.sec);
        writer_qos.lifespan().duration.nanosec = lifespan.nsec;
      }

      writer_qos.data_sharing().automatic();
      writer_qos.properties().properties().emplace_back("fastdds.intraprocess_delivery", "true");
      writer_ = publisher_->create_datawriter(topic_, writer_qos, &listener_, eprosima::fastdds::dds::StatusMask::all());
      if (!writer_)
      {
        participant_->delete_publisher(publisher_); // Cleanup on failure
        participant_->delete_topic(topic_);         // Cleanup on failure
        throw std::runtime_error("Failed to create datawriter");
      }
    }

    ~Publisher()
    {
      if (writer_ != nullptr)
      {
        publisher_->delete_datawriter(writer_);
      }
      if (publisher_ != nullptr)
      {
        participant_->delete_publisher(publisher_);
      }
      if (topic_ != nullptr && topic_owned_)
      {
        participant_->delete_topic(topic_);
      }
    }

    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    Publisher(Publisher &&) = default;
    Publisher &operator=(Publisher &&) = default;

    void publish(const std::shared_ptr<T> &message) const
    {
      writer_->write(message.get());
    }

    void publish(T &message) const
    {
      writer_->write(&message);
    }

    void publish(const T &message) const
    {
      writer_->write(const_cast<T *>(&message));
    }

    // Publish a loaned message (zero-copy publish)
    void publish(LoanedMessage<T> &&loaned_message);

    int32_t get_subscriber_count()
    {
      return listener_.count;
    }

    // Alias for rclcpp compatibility
    int32_t get_subscription_count()
    {
      return get_subscriber_count();
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

    // Zero-copy API: borrow a loaned message
    // Returns a LoanedMessage that can be used for zero-copy publishing
    LoanedMessage<T> borrow_loaned_message();

    // Check if the publisher can loan messages (zero-copy support)
    bool can_loan_messages() const
    {
      // Check if data sharing is enabled and the type is fixed size
      // For simplicity, we return true as FastDDS data_sharing is enabled
      return true;
    }

    // Get the data writer (for internal use)
    dds::DataWriter *get_writer() const { return writer_; }

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    dds::DomainParticipant *participant_;
    MessageType message_type_;
    dds::Topic *topic_;
    dds::Publisher *publisher_;
    dds::DataWriter *writer_;
    PublisherListener listener_;
    bool topic_owned_;
  };

  // LoanedMessage class for zero-copy publishing
  template <typename T>
  class LoanedMessage
  {
  public:
    using MessageType = T;

    // Constructor that loans a message from the publisher
    explicit LoanedMessage(Publisher<T> &publisher)
        : publisher_(&publisher), message_(nullptr), is_valid_(false)
    {
      void *sample = nullptr;
      if (publisher_->get_writer()->loan_sample(sample) == ReturnCode_t::RETCODE_OK)
      {
        message_ = static_cast<T *>(sample);
        is_valid_ = true;
      }
      else
      {
        // Fallback to heap allocation if loan fails
        message_ = new T();
        is_valid_ = true;
        loaned_ = false;
      }
    }

    // Move constructor
    LoanedMessage(LoanedMessage &&other) noexcept
        : publisher_(other.publisher_), message_(other.message_), is_valid_(other.is_valid_), loaned_(other.loaned_)
    {
      other.publisher_ = nullptr;
      other.message_ = nullptr;
      other.is_valid_ = false;
    }

    // Move assignment
    LoanedMessage &operator=(LoanedMessage &&other) noexcept
    {
      if (this != &other)
      {
        release();
        publisher_ = other.publisher_;
        message_ = other.message_;
        is_valid_ = other.is_valid_;
        loaned_ = other.loaned_;
        other.publisher_ = nullptr;
        other.message_ = nullptr;
        other.is_valid_ = false;
      }
      return *this;
    }

    // Destructor - returns the loan if not published
    ~LoanedMessage()
    {
      release();
    }

    // Non-copyable
    LoanedMessage(const LoanedMessage &) = delete;
    LoanedMessage &operator=(const LoanedMessage &) = delete;

    // Check if the loaned message is valid
    bool is_valid() const { return is_valid_ && message_ != nullptr; }

    // Access the message
    T &get() { return *message_; }
    const T &get() const { return *message_; }

    T *operator->() { return message_; }
    const T *operator->() const { return message_; }

    T &operator*() { return *message_; }
    const T &operator*() const { return *message_; }

    // Release ownership (called when publishing)
    T *release_ownership()
    {
      T *msg = message_;
      message_ = nullptr;
      is_valid_ = false;
      return msg;
    }

    // Check if this was a true loan (vs heap allocation fallback)
    bool is_loaned() const { return loaned_; }

    // Get the publisher
    Publisher<T> *get_publisher() const { return publisher_; }

  private:
    void release()
    {
      if (message_ != nullptr)
      {
        if (loaned_ && publisher_ != nullptr)
        {
          void *sample = message_;
          publisher_->get_writer()->discard_loan(sample);
        }
        else if (!loaned_)
        {
          delete message_;
        }
        message_ = nullptr;
      }
      is_valid_ = false;
    }

    Publisher<T> *publisher_;
    T *message_;
    bool is_valid_;
    bool loaned_ = true;
  };

  // Implementation of Publisher methods that depend on LoanedMessage
  template <typename T>
  LoanedMessage<T> Publisher<T>::borrow_loaned_message()
  {
    return LoanedMessage<T>(*this);
  }

  template <typename T>
  void Publisher<T>::publish(LoanedMessage<T> &&loaned_message)
  {
    if (!loaned_message.is_valid())
    {
      throw std::runtime_error("Cannot publish invalid loaned message");
    }

    if (loaned_message.is_loaned())
    {
      // True zero-copy publish
      T *msg = loaned_message.release_ownership();
      writer_->write(msg);
    }
    else
    {
      // Fallback publish (was heap allocated)
      T *msg = loaned_message.release_ownership();
      writer_->write(msg);
      delete msg;
    }
  }
} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
