#ifndef LWRCL_PUBLISHER_HPP_
#define LWRCL_PUBLISHER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <vsomeip/vsomeip.hpp>
#include "qos.hpp"
#include "vsomeip_header.hpp"
#include "serialized_message.hpp"

namespace lwrcl
{

  // Forward declaration – Serialization<T> is defined in lwrcl.hpp
  template <typename T>
  class Serialization;

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
    Publisher(std::shared_ptr<vsomeip::application> app,
             const std::string &topic_name,
             const QoS &qos)
        : IPublisher(),
          std::enable_shared_from_this<Publisher<T>>(),
          app_(app),
          topic_name_(topic_name),
          service_id_(someip_id::topic_to_service_id(topic_name)),
          instance_id_(someip_id::default_instance_id()),
          event_id_(someip_id::default_event_id()),
          eventgroup_id_(someip_id::default_eventgroup_id()),
          stopped_(false),
          subscriber_count_(0)
    {
      (void)qos; // QoS is not used in vsomeip transport layer
      try
      {
        // Register subscription handler to track subscriber count
        // Use vsomeip_sec_client_t-aware API (non-deprecated)
        app_->register_subscription_handler(
            service_id_, instance_id_, eventgroup_id_,
            [this](vsomeip::client_t, const vsomeip_sec_client_t*,
                   const std::string&, bool _subscribed) -> bool
            {
              if (stopped_.load())
              {
                return false;
              }
              if (_subscribed)
              {
                subscriber_count_.fetch_add(1);
              }
              else
              {
                // Use CAS loop to decrement only when > 0, avoiding a TOCTOU
                // race between a separate load() + fetch_sub() pair.
                int32_t prev = subscriber_count_.load();
                while (prev > 0 &&
                       !subscriber_count_.compare_exchange_weak(prev, prev - 1))
                {
                }
              }
              return true; // accept subscription
            });

        // Offer the service
        app_->offer_service(service_id_, instance_id_);

        // Offer the event for pub/sub
        std::set<vsomeip::eventgroup_t> eventgroups;
        eventgroups.insert(eventgroup_id_);
        app_->offer_event(
            service_id_, instance_id_, event_id_, eventgroups,
            vsomeip::event_type_e::ET_EVENT,
            std::chrono::milliseconds::zero(),
            false,  // change_resets_cycle
            true,   // update_on_change
            nullptr // epsilon_change_func
        );
      }
      catch (const std::exception &e)
      {
        throw std::runtime_error("Failed to create vsomeip publisher: " + std::string(e.what()));
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
      if (stopped_.load() || !app_ || !message)
        return;
      publish_impl(*message);
    }

    void publish(T &message) const
    {
      if (stopped_.load())
        return;
      publish_impl(message);
    }

    void publish(const T &message) const
    {
      if (stopped_.load())
        return;
      publish_impl(message);
    }

    void publish(LoanedMessage<T> &&loaned_message);

    int32_t get_subscriber_count() override
    {
      return subscriber_count_.load();
    }

    int32_t get_subscription_count()
    {
      return get_subscriber_count();
    }

    std::string get_topic_name() const
    {
      return topic_name_;
    }

    bool can_loan_messages() const
    {
      return true;
    }

    LoanedMessage<T> borrow_loaned_message();

    using SharedPtr = std::shared_ptr<Publisher<T>>;

  private:
    void publish_impl(const T &message) const
    {
      if (stopped_.load() || !app_)
        return;

      // Serialize the message using CycloneDDS CDR
      SerializedMessage serialized;
      Serialization<T>::serialize_message(&message, &serialized);

      // Create vsomeip payload from serialized bytes
      auto payload = vsomeip::runtime::get()->create_payload();
      auto &raw = serialized.get_rcl_serialized_message();
      payload->set_data(
          reinterpret_cast<const vsomeip::byte_t *>(raw.buffer),
          static_cast<vsomeip::length_t>(raw.length));

      // Notify all subscribers
      app_->notify(service_id_, instance_id_, event_id_, payload);
    }

    void cleanup()
    {
      if (stopped_.exchange(true))
      {
        return;
      }
      if (app_)
      {
        // Unregister the subscription handler first: it captures `this`, so
        // leaving it registered would let vsomeip call into a destroyed object.
        app_->unregister_subscription_handler(service_id_, instance_id_, eventgroup_id_);
        app_->stop_offer_event(service_id_, instance_id_, event_id_);
        app_->stop_offer_service(service_id_, instance_id_);
      }
    }

    std::shared_ptr<vsomeip::application> app_;
    std::string topic_name_;
    vsomeip::service_t service_id_;
    vsomeip::instance_t instance_id_;
    vsomeip::event_t event_id_;
    vsomeip::eventgroup_t eventgroup_id_;
    std::atomic<bool> stopped_;
    std::atomic<int32_t> subscriber_count_;
  };

  template <typename T>
  class LoanedMessage
  {
  public:
    using MessageType = T;

    explicit LoanedMessage(Publisher<T> &publisher)
        : publisher_(&publisher),
          message_(new T()),
          is_valid_(message_ != nullptr)
    {
    }

    LoanedMessage(LoanedMessage &&other) noexcept
        : publisher_(other.publisher_),
          message_(std::move(other.message_)),
          is_valid_(other.is_valid_)
    {
      other.publisher_ = nullptr;
      other.is_valid_ = false;
    }

    LoanedMessage &operator=(LoanedMessage &&other) noexcept
    {
      if (this != &other)
      {
        publisher_ = other.publisher_;
        message_ = std::move(other.message_);
        is_valid_ = other.is_valid_;
        other.publisher_ = nullptr;
        other.is_valid_ = false;
      }
      return *this;
    }

    LoanedMessage(const LoanedMessage &) = delete;
    LoanedMessage &operator=(const LoanedMessage &) = delete;

    bool is_valid() const
    {
      return is_valid_ && message_ != nullptr;
    }

    bool is_loaned() const
    {
      return false;
    }

    T &get()
    {
      if (!is_valid())
      {
        throw std::runtime_error("Attempting to access invalid loaned message");
      }
      return *message_;
    }

    const T &get() const
    {
      if (!is_valid())
      {
        throw std::runtime_error("Attempting to access invalid loaned message");
      }
      return *message_;
    }

    T *operator->() { return &get(); }
    const T *operator->() const { return &get(); }
    T &operator*() { return get(); }
    const T &operator*() const { return get(); }

    std::unique_ptr<T> release_heap_sample()
    {
      is_valid_ = false;
      return std::move(message_);
    }

  private:
    Publisher<T> *publisher_;
    std::unique_ptr<T> message_;
    bool is_valid_;
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

    auto sample = loaned_message.release_heap_sample();
    if (!sample)
    {
      throw std::runtime_error("Failed to publish loaned message: heap sample is invalid");
    }
    publish_impl(*sample);
  }

} // namespace lwrcl

#endif // LWRCL_PUBLISHER_HPP_
