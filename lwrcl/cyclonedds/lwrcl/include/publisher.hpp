#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <atomic>

#include <dds/dds.hpp>
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
        
        // Create topic
        dds::topic::qos::TopicQos topic_qos;
        
        // Create topic directly (Cyclone DDS doesn't require topic reuse checking)
        topic_ = std::make_shared<dds::topic::Topic<T>>(*participant_, topic_name, topic_qos);
        topic_owned_ = true;

        // Configure DataWriter QoS
        dds::pub::qos::DataWriterQos writer_qos;
        
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
        }
        
        // Set durability policy
        if (qos.get_durability() == QoS::DurabilityPolicy::VOLATILE) {
          writer_qos << dds::core::policy::Durability::Volatile();
        } else {
          writer_qos << dds::core::policy::Durability::TransientLocal();
        }

        // Create DataWriter with a listener so subscriber count can be tracked without extra calls.
        writer_ = std::make_shared<dds::pub::DataWriter<T>>(
            *publisher_, *topic_, writer_qos, &listener_,
            dds::core::status::StatusMask::publication_matched());
        
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
      return listener_.count.load();
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

    // Zero-copy API: borrow a loaned message.
    // Tries native CycloneDDS writer-loan first, then falls back to heap.
    LoanedMessage<T> borrow_loaned_message();

    // Check if the publisher can loan messages (zero-copy support)
    // API is always available (native loan or safe fallback).
    bool can_loan_messages() const
    {
      return true;
    }

    // Check whether native CycloneDDS writer loan is currently available.
    bool is_native_loan_supported() const
    {
      if (!writer_)
      {
        return false;
      }
      try
      {
        return writer_->delegate()->is_loan_supported();
      }
      catch (...)
      {
        return false;
      }
    }

    // Get the data writer (for internal use)
    dds::pub::DataWriter<T> *get_writer() const { return writer_.get(); }

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    void cleanup()
    {
      // Cyclone DDS uses RAII, so explicit cleanup is generally not needed
      // However, we can reset shared_ptrs to ensure proper destruction order
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
    PublisherListener<T> listener_;
    bool topic_owned_;
  };

  // =========================================================================
  // LoanedMessage — zero-copy publishing helper (CycloneDDS backend)
  //
  // Uses native CycloneDDS writer-loan when available (iceoryx SHM enabled
  // and compatible type), otherwise falls back to heap allocation.
  // =========================================================================
  template <typename T>
  class LoanedMessage
  {
  public:
    using MessageType = T;

    // Constructor — tries native writer-loan, falls back to heap allocation.
    explicit LoanedMessage(Publisher<T> &publisher)
        : publisher_(&publisher), message_(nullptr), is_valid_(false), loaned_(false)
    {
      if (publisher_ != nullptr && publisher_->get_writer() != nullptr &&
          publisher_->is_native_loan_supported())
      {
        try
        {
          message_ = &publisher_->get_writer()->delegate()->loan_sample();
          is_valid_ = true;
          loaned_ = true;
          return;
        }
        catch (...)
        {
          // Fallback below.
        }
      }

      message_ = new T();
      is_valid_ = true;
      loaned_ = false;
    }

    // Move constructor
    LoanedMessage(LoanedMessage &&other) noexcept
        : publisher_(other.publisher_), message_(other.message_),
          is_valid_(other.is_valid_), loaned_(other.loaned_)
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

    // Check if this is a native CycloneDDS loan (vs heap fallback).
    bool is_loaned() const { return loaned_; }

    // Get the publisher
    Publisher<T> *get_publisher() const { return publisher_; }

  private:
    void release()
    {
      if (message_ != nullptr)
      {
        if (!loaned_)
        {
          delete message_;
        }
        else if (publisher_ != nullptr && publisher_->get_writer() != nullptr)
        {
          try
          {
            publisher_->get_writer()->delegate()->return_loan(*message_);
          }
          catch (...)
          {
            // Best-effort release in destructor path.
          }
        }
        message_ = nullptr;
      }
      is_valid_ = false;
    }

    Publisher<T> *publisher_;
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

    T *msg = loaned_message.release_ownership();
    if (writer_ && msg)
    {
      try
      {
        writer_->write(*msg);
      }
      catch (...)
      {
        if (loaned_message.is_loaned())
        {
          try
          {
            writer_->delegate()->return_loan(*msg);
          }
          catch (...)
          {
            // Best-effort release before rethrow.
          }
        }
        else
        {
          delete msg;
        }
        throw;
      }
    }
    else if (msg)
    {
      if (loaned_message.is_loaned())
      {
        try
        {
          if (writer_)
          {
            writer_->delegate()->return_loan(*msg);
          }
        }
        catch (...)
        {
          // Best-effort cleanup.
        }
      }
      else
      {
        delete msg;
      }
      throw std::runtime_error("Failed to publish loaned message: DataWriter is null");
    }

    if (!loaned_message.is_loaned())
    {
      delete msg;
    }
  }
} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
