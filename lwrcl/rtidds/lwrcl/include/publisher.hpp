#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>
#include <atomic>

#include <dds/dds.hpp>
#include <dds/topic/find.hpp>
#include <rti/zcopy/rtizcopy.hpp>
#include "qos.hpp"

namespace lwrcl
{

  // Forward declaration for LoanedMessage
  template <typename T>
  class LoanedMessage;

  template <typename T>
  class PublisherListener : public dds::pub::NoOpDataWriterListener<T>
  {
  public:
    void on_publication_matched(dds::pub::DataWriter<T>& writer, const dds::core::status::PublicationMatchedStatus& status) override
    {
      (void)writer; // Unused parameter
      count = status.current_count();
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
    Publisher(dds::domain::DomainParticipant *participant, const std::string &topic_name, const QoS &qos)
        : IPublisher(),
          std::enable_shared_from_this<Publisher<T>>(),
          participant_(participant),
          topic_(nullptr),
          publisher_(nullptr),
          writer_(nullptr),
          topic_owned_(false)
    {
      try {
        // Create publisher
        publisher_ = std::make_shared<dds::pub::Publisher>(*participant_);

        // Start from the participant's configured default; RTI's QoS default
        // constructor produces an empty delegate.
        auto existing_topic =
            dds::topic::find<dds::topic::Topic<T>>(*participant_, topic_name);
        if (existing_topic != dds::core::null) {
          topic_ = std::make_shared<dds::topic::Topic<T>>(existing_topic);
        } else {
          auto topic_qos = participant_->default_topic_qos();
          topic_ = std::make_shared<dds::topic::Topic<T>>(
              *participant_, topic_name, topic_qos);
          topic_owned_ = true;
        }

        // Configure DataWriter QoS
        auto writer_qos = publisher_->default_datawriter_qos();

        // Set history policy
        if (qos.get_history() == QoS::HistoryPolicy::KEEP_ALL) {
          writer_qos << dds::core::policy::History::KeepAll();
        } else {
          writer_qos << dds::core::policy::History::KeepLast(qos.get_depth());
        }

        // Set reliability policy
        if (qos.get_reliability() == QoS::ReliabilityPolicy::BEST_EFFORT) {
          writer_qos << dds::core::policy::Reliability::BestEffort();
        } else {
          writer_qos << dds::core::policy::Reliability::Reliable();
          // RTI requires asynchronous publishing when a reliable sample is
          // larger than a transport message and must be fragmented.
          writer_qos << rti::core::policy::PublishMode::Asynchronous();
        }

        // Set durability policy
        if (qos.get_durability() == QoS::DurabilityPolicy::VOLATILE) {
          writer_qos << dds::core::policy::Durability::Volatile();
        } else {
          writer_qos << dds::core::policy::Durability::TransientLocal();
        }

        // Query match status on demand. Avoid retaining a raw listener pointer
        // inside an RTI entity beyond the C++ wrapper's lifetime.
        writer_ = std::make_shared<dds::pub::DataWriter<T>>(
            *publisher_, *topic_, writer_qos);

      } catch (const dds::core::Exception& e) {
        cleanup();
        throw std::runtime_error("Failed to create publisher: " + std::string(e.what()));
      }
    }

    ~Publisher()
    {
      cleanup();
    }

    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    Publisher(Publisher &&) = delete;
    Publisher &operator=(Publisher &&) = delete;

    void publish(const std::shared_ptr<T> &message) const
    {
      if (writer_ && message) {
        writer_->write(*message);
      }
    }

    void publish(T &message) const
    {
      if (writer_) {
        writer_->write(message);
      }
    }

    void publish(const T &message) const
    {
      if (writer_) {
        writer_->write(message);
      }
    }

    // Publish a loaned message (zero-copy publish)
    void publish(LoanedMessage<T> &&loaned_message);

    int32_t get_subscriber_count() override
    {
      if (!writer_) {
        return 0;
      }
      try {
        return writer_->publication_matched_status().current_count();
      } catch (const dds::core::Exception &) {
        return 0;
      }
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
        return topic_->name();
      }
      return "";
    }

    // Zero-copy API: borrow a native RTI writer loan for generated SHMEM_REF
    // types, or use the safe heap fallback for other types/resources.
    LoanedMessage<T> borrow_loaned_message();

    // Check if the publisher can loan messages (zero-copy support)
    // API is always available (native loan or safe fallback).
    bool can_loan_messages() const
    {
      return true;
    }

    // Check whether a native RTI writer loan is currently available.
    bool is_native_loan_supported() const
    {
      return writer_ != nullptr &&
          rti::zcopy::topic::is_zcopy_type_only<T>::value;
    }

    // Get the data writer (for internal use)
    dds::pub::DataWriter<T> *get_writer() const { return writer_.get(); }

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    friend class LoanedMessage<T>;

    using NativeLoanTag = std::integral_constant<
        bool, rti::zcopy::topic::is_zcopy_type_only<T>::value>;

    T *try_native_loan(std::true_type)
    {
      if (!writer_) {
        return nullptr;
      }
      try {
        return writer_->delegate()->get_loan();
      } catch (const dds::core::Exception &) {
        // Resource exhaustion must not make the portable loan API unusable.
        return nullptr;
      }
    }

    T *try_native_loan(std::false_type)
    {
      return nullptr;
    }

    T *try_native_loan()
    {
      return try_native_loan(NativeLoanTag{});
    }

    void discard_native_loan(T *sample, std::true_type) const
    {
      writer_->delegate()->discard_loan(*sample);
    }

    void discard_native_loan(T *, std::false_type) const
    {
    }

    void discard_native_loan(T *sample) const
    {
      discard_native_loan(sample, NativeLoanTag{});
    }

    void cleanup()
    {
      // Reset shared_ptrs explicitly to preserve RTI entity destruction order.
      if (writer_) {
        writer_.reset();
      }
      if (publisher_) {
        publisher_.reset();
      }
      if (topic_ && topic_owned_) {
        topic_.reset();
      }
    }

    dds::domain::DomainParticipant *participant_;
    std::shared_ptr<dds::topic::Topic<T>> topic_;
    std::shared_ptr<dds::pub::Publisher> publisher_;
    std::shared_ptr<dds::pub::DataWriter<T>> writer_;
    bool topic_owned_;
  };

  // =========================================================================
  // LoanedMessage — loan-compatible publishing helper (RTI DDS backend)
  //
  // Fixed-size generated ROS types use RTI's SHMEM_REF writer loan. Types with
  // strings/sequences, or a temporarily exhausted writer pool, use heap memory.
  // =========================================================================
  template <typename T>
  class LoanedMessage
  {
  public:
    using MessageType = T;

    // Constructor — tries native writer-loan, falls back to heap allocation.
    explicit LoanedMessage(Publisher<T> &publisher)
        : publisher_(&publisher), writer_(publisher.writer_), message_(nullptr),
          is_valid_(false), loaned_(false)
    {
      message_ = publisher.try_native_loan();
      loaned_ = message_ != nullptr;
      if (message_ == nullptr) {
        message_ = new T();
      }
      is_valid_ = true;
    }

    // Move constructor
    LoanedMessage(LoanedMessage &&other) noexcept
        : publisher_(other.publisher_), writer_(std::move(other.writer_)),
          message_(other.message_),
          is_valid_(other.is_valid_), loaned_(other.loaned_)
    {
      other.publisher_ = nullptr;
      other.message_ = nullptr;
      other.is_valid_ = false;
      other.loaned_ = false;
    }

    // Move assignment
    LoanedMessage &operator=(LoanedMessage &&other) noexcept
    {
      if (this != &other)
      {
        release();
        publisher_ = other.publisher_;
        writer_ = std::move(other.writer_);
        message_ = other.message_;
        is_valid_ = other.is_valid_;
        loaned_ = other.loaned_;
        other.publisher_ = nullptr;
        other.message_ = nullptr;
        other.is_valid_ = false;
        other.loaned_ = false;
      }
      return *this;
    }

    // Destructor — frees the message if not published
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

    // Check if this is a native RTI loan (vs heap fallback).
    bool is_loaned() const { return loaned_; }

    // Get the publisher
    Publisher<T> *get_publisher() const { return publisher_; }

  private:
    using NativeLoanTag = std::integral_constant<
        bool, rti::zcopy::topic::is_zcopy_type_only<T>::value>;

    void discard_native_loan(std::true_type)
    {
      writer_->delegate()->discard_loan(*message_);
    }

    void discard_native_loan(std::false_type)
    {
    }

    void release()
    {
      if (message_ != nullptr)
      {
        if (loaned_ && writer_) {
          try {
            discard_native_loan(NativeLoanTag{});
          } catch (const dds::core::Exception &) {
            // Destructors must not throw. RTI still owns native loan storage.
          }
        } else {
          delete message_;
        }
        message_ = nullptr;
      }
      is_valid_ = false;
      loaned_ = false;
    }

    Publisher<T> *publisher_;
    std::shared_ptr<dds::pub::DataWriter<T>> writer_;
    T *message_;
    bool is_valid_;
    bool loaned_ = false;
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

    if (writer_ == nullptr)
    {
      throw std::runtime_error("Failed to publish loaned message: DataWriter is null");
    }

    if (loaned_message.get_publisher() != this)
    {
      throw std::runtime_error(
          "Cannot publish a loaned message with a different publisher");
    }

    const bool native_loan = loaned_message.is_loaned();
    T *msg = loaned_message.release_ownership();
    if (msg)
    {
      try
      {
        writer_->write(*msg);
      }
      catch (...)
      {
        if (native_loan) {
          try {
            discard_native_loan(msg);
          } catch (const dds::core::Exception &) {
          }
        } else {
          delete msg;
        }
        throw;
      }
    }

    // A successful write transfers a native loan back to RTI.
    if (!native_loan)
    {
      delete msg;
    }
  }
} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
