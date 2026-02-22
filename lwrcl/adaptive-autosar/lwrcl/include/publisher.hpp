#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "adaptive_autosar_header.hpp"
#include "lwrcl_autosar_proxy_skeleton.hpp"
#include "qos.hpp"

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
          topic_name_(topic_name),
          subscriber_count_(0)
    {
      (void)participant;
      (void)qos; // QoS is fixed in Adaptive AUTOSAR DDS layer

      try
      {
        skeleton_ = std::make_unique<lwrcl::autosar_generated::TopicEventSkeleton<T>>(topic_name_);

        auto offer_service_result = skeleton_->OfferService();
        if (!offer_service_result.HasValue())
        {
          throw std::runtime_error("Failed to offer AUTOSAR event service.");
        }
        auto offer_event_result = skeleton_->Event.Offer();
        if (!offer_event_result.HasValue())
        {
          throw std::runtime_error("Failed to offer AUTOSAR event.");
        }
      }
      catch (const std::exception &e)
      {
        throw std::runtime_error("Failed to create publisher: " + std::string(e.what()));
      }
    }

    ~Publisher()
    {
      if (skeleton_)
      {
        skeleton_->Event.StopOffer();
        skeleton_->StopOfferService();
      }
    }

    Publisher(const Publisher &) = delete;
    Publisher &operator=(const Publisher &) = delete;
    Publisher(Publisher &&) = delete;
    Publisher &operator=(Publisher &&) = delete;

    void publish(const std::shared_ptr<T> &message) const
    {
      if (message)
      {
        publish_impl(*message);
      }
    }

    void publish(T &message) const
    {
      publish_impl(message);
    }

    void publish(const T &message) const
    {
      publish_impl(message);
    }

    int32_t get_subscriber_count() override
    {
      return subscriber_count_.load();
    }

    // Alias for rclcpp compatibility
    int32_t get_subscription_count()
    {
      return get_subscriber_count();
    }

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
    void publish_impl(const T &message) const
    {
        if (skeleton_)
        {
        skeleton_->Event.Send(message);
        }
      }

    std::string topic_name_;
    std::unique_ptr<lwrcl::autosar_generated::TopicEventSkeleton<T>> skeleton_;
    mutable std::atomic<int32_t> subscriber_count_;
  };

} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
