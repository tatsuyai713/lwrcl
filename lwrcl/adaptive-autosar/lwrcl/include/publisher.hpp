#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "adaptive_autosar_header.hpp"
#include "lwrcl_autosar_proxy_skeleton.hpp"
#include "qos.hpp"

namespace lwrcl
{
  template <typename T>
  class LoanedMessage;

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
        skeleton_ = std::make_unique<autosar_generated::TopicEventSkeleton<T>>(topic_name_);

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

    // Publish a loaned message (zero-copy when supported by runtime binding)
    void publish(LoanedMessage<T> &&loaned_message);

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

    // Loaned API is always available; runtime binding decides copy/zero-copy path.
    LoanedMessage<T> borrow_loaned_message();

    bool can_loan_messages() const
    {
      return true;
    }

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    friend class LoanedMessage<T>;

    bool try_allocate_loaned_sample(ara::com::SampleAllocateePtr<T> &out_sample) const
    {
      if (!skeleton_)
      {
        return false;
      }

      auto alloc_result = skeleton_->Event.Allocate();
      if (!alloc_result.HasValue())
      {
        return false;
      }

      out_sample = std::move(alloc_result).Value();
      return static_cast<bool>(out_sample);
    }

    void publish_impl(const T &message) const
    {
      if (skeleton_)
      {
        skeleton_->Event.Send(message);
      }
    }

    std::string topic_name_;
    std::unique_ptr<autosar_generated::TopicEventSkeleton<T>> skeleton_;
    mutable std::atomic<int32_t> subscriber_count_;
  };

  template <typename T>
  class LoanedMessage
  {
  public:
    using MessageType = T;

    explicit LoanedMessage(Publisher<T> &publisher)
        : publisher_(&publisher),
          message_(nullptr),
          is_valid_(false),
          backend_allocated_(false)
    {
      if (publisher_ != nullptr && publisher_->try_allocate_loaned_sample(loaned_sample_))
      {
        message_ = loaned_sample_.Get();
        is_valid_ = message_ != nullptr;
        backend_allocated_ = is_valid_;
        return;
      }

      heap_message_ = std::unique_ptr<T>(new T());
      message_ = heap_message_.get();
      is_valid_ = message_ != nullptr;
      backend_allocated_ = false;
    }

    LoanedMessage(LoanedMessage &&other) noexcept
        : publisher_(other.publisher_),
          loaned_sample_(std::move(other.loaned_sample_)),
          heap_message_(std::move(other.heap_message_)),
          message_(other.message_),
          is_valid_(other.is_valid_),
          backend_allocated_(other.backend_allocated_)
    {
      other.publisher_ = nullptr;
      other.message_ = nullptr;
      other.is_valid_ = false;
      other.backend_allocated_ = false;
    }

    LoanedMessage &operator=(LoanedMessage &&other) noexcept
    {
      if (this != &other)
      {
        publisher_ = other.publisher_;
        loaned_sample_ = std::move(other.loaned_sample_);
        heap_message_ = std::move(other.heap_message_);
        message_ = other.message_;
        is_valid_ = other.is_valid_;
        backend_allocated_ = other.backend_allocated_;

        other.publisher_ = nullptr;
        other.message_ = nullptr;
        other.is_valid_ = false;
        other.backend_allocated_ = false;
      }
      return *this;
    }

    ~LoanedMessage() = default;

    LoanedMessage(const LoanedMessage &) = delete;
    LoanedMessage &operator=(const LoanedMessage &) = delete;

    bool is_valid() const
    {
      return is_valid_ && message_ != nullptr;
    }

    // true when allocated via ara::com::SkeletonEvent::Allocate().
    bool is_loaned() const
    {
      return backend_allocated_;
    }

    T &get()
    {
      if (!is_valid_ || message_ == nullptr)
        throw std::runtime_error("Attempting to access invalid loaned message");
      return *message_;
    }

    const T &get() const
    {
      if (!is_valid_ || message_ == nullptr)
        throw std::runtime_error("Attempting to access invalid loaned message");
      return *message_;
    }

    T *operator->()
    {
      return message_;
    }

    const T *operator->() const
    {
      return message_;
    }

    T &operator*()
    {
      return *message_;
    }

    const T &operator*() const
    {
      return *message_;
    }

    T *release_heap_sample()
    {
      if (backend_allocated_)
      {
        return nullptr;
      }

      is_valid_ = false;
      message_ = nullptr;
      return heap_message_.release();
    }

    ara::com::SampleAllocateePtr<T> release_loaned_sample()
    {
      if (!backend_allocated_)
      {
        return ara::com::SampleAllocateePtr<T>();
      }

      is_valid_ = false;
      message_ = nullptr;
      backend_allocated_ = false;
      return std::move(loaned_sample_);
    }

  private:
    Publisher<T> *publisher_;
    ara::com::SampleAllocateePtr<T> loaned_sample_;
    std::unique_ptr<T> heap_message_;
    T *message_;
    bool is_valid_;
    bool backend_allocated_;
  };

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

    if (!skeleton_)
    {
      throw std::runtime_error("Failed to publish loaned message: skeleton is null");
    }

    if (loaned_message.is_loaned())
    {
      // Generated bindings currently route SendAllocated as raw bytes.
      // For non-trivially-copyable ROS messages, use safe typed Send().
      if (!std::is_trivially_copyable<T>::value)
      {
        skeleton_->Event.Send(loaned_message.get());
        return;
      }

      auto sample = loaned_message.release_loaned_sample();
      if (!sample)
      {
        throw std::runtime_error("Failed to publish loaned message: allocated sample is invalid");
      }
      skeleton_->Event.Send(std::move(sample));
      return;
    }

    std::unique_ptr<T> sample_guard(loaned_message.release_heap_sample());
    if (!sample_guard)
    {
      throw std::runtime_error("Failed to publish loaned message: heap sample is invalid");
    }
    skeleton_->Event.Send(*sample_guard);
  }

} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
