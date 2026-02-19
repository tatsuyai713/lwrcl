#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <atomic>

#include <ara/com/dds/dds_pubsub.h>
#include "qos.hpp"
#include "adaptive_autosar_header.hpp"

namespace lwrcl
{

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
    Publisher(AutosarDomainParticipant *participant, const std::string &topic_name, const QoS &qos)
        : IPublisher(),
          std::enable_shared_from_this<Publisher<T>>(),
          topic_name_(topic_name)
    {
      (void)qos; // QoS is fixed in Adaptive AUTOSAR DDS layer
      try {
        publisher_ = std::make_unique<ara::com::dds::DdsPublisher<T>>(
            topic_name, participant->domain_id());
        if (!publisher_->IsBindingActive()) {
          throw std::runtime_error("Adaptive AUTOSAR publisher binding not active for topic: " + topic_name);
        }
      } catch (const std::exception &e) {
        throw std::runtime_error("Failed to create publisher: " + std::string(e.what()));
      }
    }

    ~Publisher() = default;

    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    Publisher(Publisher &&) = delete;
    Publisher &operator=(Publisher &&) = delete;

    void publish(const std::shared_ptr<T> &message) const
    {
      if (publisher_ && message) {
        publisher_->Write(*message);
      }
    }

    void publish(T &message) const
    {
      if (publisher_) {
        publisher_->Write(message);
      }
    }

    void publish(const T &message) const
    {
      if (publisher_) {
        publisher_->Write(message);
      }
    }

    int32_t get_subscriber_count() override
    {
      if (publisher_) {
        auto result = publisher_->GetMatchedSubscriptionCount();
        if (result.HasValue()) {
          return result.Value();
        }
      }
      return 0;
    }

    // Alias for rclcpp compatibility
    int32_t get_subscription_count()
    {
      return get_subscriber_count();
    }

    // Get the topic name
    std::string get_topic_name() const
    {
      return topic_name_;
    }

    // No zero-copy support in Adaptive AUTOSAR backend
    bool can_loan_messages() const
    {
      return false;
    }

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    std::string topic_name_;
    std::unique_ptr<ara::com::dds::DdsPublisher<T>> publisher_;
  };

} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
