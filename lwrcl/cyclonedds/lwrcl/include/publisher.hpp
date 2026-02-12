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

    // Check if the publisher can loan messages (zero-copy support)
    // CycloneDDS does not support zero-copy loaned messages in this wrapper
    bool can_loan_messages() const
    {
      return false;
    }

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
} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
