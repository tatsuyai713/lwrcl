#ifndef LWRCL_QOS_HPP
#define LWRCL_QOS_HPP

#include <chrono>
#include <cstddef>
#include <stdexcept>

// Custom QoS history policy enumeration
enum class RMWQoSHistoryPolicy
{
  KEEP_LAST,
  KEEP_ALL
};

// Custom QoS reliability policy enumeration
enum class RMWQoSReliabilityPolicy
{
  BEST_EFFORT,
  RELIABLE
};

// Custom QoS durability policy enumeration
enum class RMWQoSDurabilityPolicy
{
  VOLATILE,
  TRANSIENT_LOCAL
};

// Custom QoS liveliness policy enumeration
enum class RMWQoSLivelinessPolicy
{
  AUTOMATIC,
  MANUAL_BY_TOPIC
};

// Duration structure for QoS settings (deadline, lifespan, liveliness_lease_duration)
struct RMWDuration
{
  int64_t sec = 0;
  uint32_t nsec = 0;

  static constexpr int64_t INFINITE_SEC = 0x7FFFFFFF;
  static constexpr uint32_t INFINITE_NSEC = 0xFFFFFFFF;

  RMWDuration() = default;
  RMWDuration(int64_t s, uint32_t ns) : sec(s), nsec(ns) {}

  template <typename Rep, typename Period>
  explicit RMWDuration(const std::chrono::duration<Rep, Period> &duration)
  {
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    sec = ns / 1000000000LL;
    nsec = static_cast<uint32_t>(ns % 1000000000LL);
  }

  bool is_infinite() const
  {
    return sec == INFINITE_SEC && nsec == INFINITE_NSEC;
  }

  static RMWDuration infinite()
  {
    return RMWDuration(INFINITE_SEC, INFINITE_NSEC);
  }
};

// Custom QoS profile structure
struct RMWQoSProfile
{
  RMWQoSProfile()
      : depth(10), history(RMWQoSHistoryPolicy::KEEP_LAST), reliability(RMWQoSReliabilityPolicy::RELIABLE), durability(RMWQoSDurabilityPolicy::VOLATILE),
        liveliness(RMWQoSLivelinessPolicy::AUTOMATIC), deadline(RMWDuration::infinite()), lifespan(RMWDuration::infinite()), liveliness_lease_duration(RMWDuration::infinite())
  {
  }

  RMWQoSProfile(
      size_t depth,
      RMWQoSHistoryPolicy history,
      RMWQoSReliabilityPolicy reliability,
      RMWQoSDurabilityPolicy durability)
      : depth(depth), history(history), reliability(reliability), durability(durability),
        liveliness(RMWQoSLivelinessPolicy::AUTOMATIC), deadline(RMWDuration::infinite()), lifespan(RMWDuration::infinite()), liveliness_lease_duration(RMWDuration::infinite())
  {
  }

  RMWQoSProfile(
      size_t depth,
      RMWQoSHistoryPolicy history,
      RMWQoSReliabilityPolicy reliability,
      RMWQoSDurabilityPolicy durability,
      RMWQoSLivelinessPolicy liveliness,
      const RMWDuration &deadline,
      const RMWDuration &lifespan,
      const RMWDuration &liveliness_lease_duration)
      : depth(depth), history(history), reliability(reliability), durability(durability),
        liveliness(liveliness), deadline(deadline), lifespan(lifespan), liveliness_lease_duration(liveliness_lease_duration)
  {
  }

  RMWQoSProfile(const RMWQoSProfile &) = default;
  RMWQoSProfile &operator=(const RMWQoSProfile &) = default;
  RMWQoSProfile(RMWQoSProfile &&) = default;
  RMWQoSProfile &operator=(RMWQoSProfile &&) = default;

  ~RMWQoSProfile() = default;

  size_t depth;
  RMWQoSHistoryPolicy history;
  RMWQoSReliabilityPolicy reliability;
  RMWQoSDurabilityPolicy durability;
  RMWQoSLivelinessPolicy liveliness;
  RMWDuration deadline;
  RMWDuration lifespan;
  RMWDuration liveliness_lease_duration;
};

namespace lwrcl
{

  class QoSInitialization
  {
  public:
    QoSInitialization(RMWQoSHistoryPolicy history, size_t depth)
        : history_(history), depth_(depth)
    {
    }

    QoSInitialization(const QoSInitialization &) = default;
    QoSInitialization &operator=(const QoSInitialization &) = default;
    QoSInitialization(QoSInitialization &&) = default;
    QoSInitialization &operator=(QoSInitialization &&) = default;

    ~QoSInitialization() = default;

    RMWQoSHistoryPolicy history_;
    size_t depth_;
  };

  class QoS
  {
  public:
    enum class HistoryPolicy
    {
      KEEP_LAST,
      KEEP_ALL
    };

    enum class ReliabilityPolicy
    {
      BEST_EFFORT,
      RELIABLE
    };

    enum class DurabilityPolicy
    {
      VOLATILE,
      TRANSIENT_LOCAL
    };

    enum class LivelinessPolicy
    {
      AUTOMATIC,
      MANUAL_BY_TOPIC
    };

    // Default constructor
    explicit QoS(size_t depth = 10)
        : depth_(depth),
          history_(HistoryPolicy::KEEP_LAST),
          reliability_(ReliabilityPolicy::RELIABLE),
          durability_(DurabilityPolicy::VOLATILE),
          liveliness_(LivelinessPolicy::AUTOMATIC),
          deadline_(RMWDuration::infinite()),
          lifespan_(RMWDuration::infinite()),
          liveliness_lease_duration_(RMWDuration::infinite())
    {
    }

    // Constructor from QoSInitialization (rclcpp compatible: QoS(KeepAll{}))
    QoS(const QoSInitialization &qos_init)
        : depth_(qos_init.depth_),
          history_(qos_init.history_ == RMWQoSHistoryPolicy::KEEP_LAST ? HistoryPolicy::KEEP_LAST : HistoryPolicy::KEEP_ALL),
          reliability_(ReliabilityPolicy::RELIABLE),
          durability_(DurabilityPolicy::VOLATILE),
          liveliness_(LivelinessPolicy::AUTOMATIC),
          deadline_(RMWDuration::infinite()),
          lifespan_(RMWDuration::infinite()),
          liveliness_lease_duration_(RMWDuration::infinite())
    {
    }

    // Constructor with HistoryPolicy and custom profile
    QoS(HistoryPolicy history, const RMWQoSProfile &custom_profile)
        : depth_(custom_profile.depth),
          history_(history),
          reliability_(custom_profile.reliability == RMWQoSReliabilityPolicy::BEST_EFFORT ? ReliabilityPolicy::BEST_EFFORT : ReliabilityPolicy::RELIABLE),
          durability_(custom_profile.durability == RMWQoSDurabilityPolicy::VOLATILE ? DurabilityPolicy::VOLATILE : DurabilityPolicy::TRANSIENT_LOCAL),
          liveliness_(custom_profile.liveliness == RMWQoSLivelinessPolicy::AUTOMATIC ? LivelinessPolicy::AUTOMATIC : LivelinessPolicy::MANUAL_BY_TOPIC),
          deadline_(custom_profile.deadline),
          lifespan_(custom_profile.lifespan),
          liveliness_lease_duration_(custom_profile.liveliness_lease_duration) {}

    QoS(const QoSInitialization qos, const RMWQoSProfile &custom_profile)
        : depth_(qos.depth_),
          history_(qos.history_ == RMWQoSHistoryPolicy::KEEP_LAST ? HistoryPolicy::KEEP_LAST : HistoryPolicy::KEEP_ALL),
          reliability_(custom_profile.reliability == RMWQoSReliabilityPolicy::BEST_EFFORT ? ReliabilityPolicy::BEST_EFFORT : ReliabilityPolicy::RELIABLE),
          durability_(custom_profile.durability == RMWQoSDurabilityPolicy::VOLATILE ? DurabilityPolicy::VOLATILE : DurabilityPolicy::TRANSIENT_LOCAL),
          liveliness_(custom_profile.liveliness == RMWQoSLivelinessPolicy::AUTOMATIC ? LivelinessPolicy::AUTOMATIC : LivelinessPolicy::MANUAL_BY_TOPIC),
          deadline_(custom_profile.deadline),
          lifespan_(custom_profile.lifespan),
          liveliness_lease_duration_(custom_profile.liveliness_lease_duration) {}

    QoS(const QoS &) = default;
    QoS &operator=(const QoS &) = default;
    QoS(QoS &&) = default;
    QoS &operator=(QoS &&) = default;

    ~QoS() = default;

    // Set HistoryPolicy to KEEP_LAST
    QoS &keep_last(size_t depth)
    {
      history_ = HistoryPolicy::KEEP_LAST;
      depth_ = depth;
      return *this;
    }

    // Set HistoryPolicy to KEEP_ALL
    QoS &keep_all()
    {
      history_ = HistoryPolicy::KEEP_ALL;
      return *this;
    }

    // Set ReliabilityPolicy
    QoS &reliability(ReliabilityPolicy policy)
    {
      reliability_ = policy;
      return *this;
    }

    // Convenience: set RELIABLE (rclcpp compatible)
    QoS &reliable()
    {
      reliability_ = ReliabilityPolicy::RELIABLE;
      return *this;
    }

    // Convenience: set BEST_EFFORT (rclcpp compatible)
    QoS &best_effort()
    {
      reliability_ = ReliabilityPolicy::BEST_EFFORT;
      return *this;
    }

    // Set DurabilityPolicy
    QoS &durability(DurabilityPolicy policy)
    {
      durability_ = policy;
      return *this;
    }

    // Convenience: set TRANSIENT_LOCAL (rclcpp compatible)
    QoS &transient_local()
    {
      durability_ = DurabilityPolicy::TRANSIENT_LOCAL;
      return *this;
    }

    // Convenience: set VOLATILE (rclcpp compatible)
    QoS &durability_volatile()
    {
      durability_ = DurabilityPolicy::VOLATILE;
      return *this;
    }

    // Set LivelinessPolicy
    QoS &liveliness(LivelinessPolicy policy)
    {
      liveliness_ = policy;
      return *this;
    }

    // Set Deadline
    template <typename Rep, typename Period>
    QoS &deadline(const std::chrono::duration<Rep, Period> &deadline)
    {
      deadline_ = RMWDuration(deadline);
      return *this;
    }

    // Set Lifespan
    template <typename Rep, typename Period>
    QoS &lifespan(const std::chrono::duration<Rep, Period> &lifespan)
    {
      lifespan_ = RMWDuration(lifespan);
      return *this;
    }

    // Set Liveliness Lease Duration
    template <typename Rep, typename Period>
    QoS &liveliness_lease_duration(const std::chrono::duration<Rep, Period> &lease_duration)
    {
      liveliness_lease_duration_ = RMWDuration(lease_duration);
      return *this;
    }

    size_t get_depth() const
    {
      return depth_;
    }

    HistoryPolicy get_history() const
    {
      return history_;
    }

    ReliabilityPolicy get_reliability() const
    {
      return reliability_;
    }

    DurabilityPolicy get_durability() const
    {
      return durability_;
    }

    LivelinessPolicy get_liveliness() const
    {
      return liveliness_;
    }

    RMWDuration get_deadline() const
    {
      return deadline_;
    }

    RMWDuration get_lifespan() const
    {
      return lifespan_;
    }

    RMWDuration get_liveliness_lease_duration() const
    {
      return liveliness_lease_duration_;
    }

    // Convert to a RMWQoSProfile
    RMWQoSProfile to_rmw_qos_profile() const
    {
      RMWQoSProfile profile;
      profile.depth = depth_;

      switch (history_)
      {
      case HistoryPolicy::KEEP_LAST:
        profile.history = RMWQoSHistoryPolicy::KEEP_LAST;
        break;
      case HistoryPolicy::KEEP_ALL:
        profile.history = RMWQoSHistoryPolicy::KEEP_ALL;
        break;
      }

      switch (reliability_)
      {
      case ReliabilityPolicy::BEST_EFFORT:
        profile.reliability = RMWQoSReliabilityPolicy::BEST_EFFORT;
        break;
      case ReliabilityPolicy::RELIABLE:
        profile.reliability = RMWQoSReliabilityPolicy::RELIABLE;
        break;
      }

      switch (durability_)
      {
      case DurabilityPolicy::VOLATILE:
        profile.durability = RMWQoSDurabilityPolicy::VOLATILE;
        break;
      case DurabilityPolicy::TRANSIENT_LOCAL:
        profile.durability = RMWQoSDurabilityPolicy::TRANSIENT_LOCAL;
        break;
      }

      switch (liveliness_)
      {
      case LivelinessPolicy::AUTOMATIC:
        profile.liveliness = RMWQoSLivelinessPolicy::AUTOMATIC;
        break;
      case LivelinessPolicy::MANUAL_BY_TOPIC:
        profile.liveliness = RMWQoSLivelinessPolicy::MANUAL_BY_TOPIC;
        break;
      }

      profile.deadline = deadline_;
      profile.lifespan = lifespan_;
      profile.liveliness_lease_duration = liveliness_lease_duration_;

      return profile;
    }

  private:
    size_t depth_;
    HistoryPolicy history_;
    ReliabilityPolicy reliability_;
    DurabilityPolicy durability_;
    LivelinessPolicy liveliness_;
    RMWDuration deadline_;
    RMWDuration lifespan_;
    RMWDuration liveliness_lease_duration_;
  };

  // Preset QoS profiles (rclcpp compatible)
  class SensorDataQoS : public QoS
  {
  public:
    SensorDataQoS() : QoS(5)
    {
      reliability(ReliabilityPolicy::BEST_EFFORT);
      durability(DurabilityPolicy::VOLATILE);
    }
  };

  class SystemDefaultsQoS : public QoS
  {
  public:
    SystemDefaultsQoS() : QoS(10)
    {
      reliability(ReliabilityPolicy::RELIABLE);
      durability(DurabilityPolicy::VOLATILE);
    }
  };

  class ServicesQoS : public QoS
  {
  public:
    ServicesQoS() : QoS(10)
    {
      reliability(ReliabilityPolicy::RELIABLE);
      durability(DurabilityPolicy::VOLATILE);
    }
  };

  class ParametersQoS : public QoS
  {
  public:
    ParametersQoS() : QoS(1000)
    {
      reliability(ReliabilityPolicy::RELIABLE);
      durability(DurabilityPolicy::VOLATILE);
    }
  };

  class ParameterEventsQoS : public QoS
  {
  public:
    ParameterEventsQoS() : QoS(1000)
    {
      reliability(ReliabilityPolicy::RELIABLE);
      durability(DurabilityPolicy::VOLATILE);
    }
  };

  class BestEffortQoS : public QoS
  {
  public:
    BestEffortQoS() : QoS(10)
    {
      reliability(ReliabilityPolicy::BEST_EFFORT);
    }
  };

  class ReliableQoS : public QoS
  {
  public:
    ReliableQoS() : QoS(10)
    {
      reliability(ReliabilityPolicy::RELIABLE);
    }
  };

  extern const RMWQoSProfile rmw_qos_profile_default;
  extern const RMWQoSProfile rmw_qos_profile_sensor_data;
  extern const RMWQoSProfile rmw_qos_profile_parameters;
  extern const RMWQoSProfile rmw_qos_profile_services_default;
  extern const RMWQoSProfile rmw_qos_profile_parameter_events;

  // rclcpp-compatible KeepLast / KeepAll --------------------------------
  // These are structs so that both QoS(KeepAll{}) and QoS(KeepAll()) work,
  // matching rclcpp's API.
  struct KeepLast
  {
    explicit KeepLast(size_t depth) : depth_(depth) {}
    operator QoSInitialization() const                     // NOLINT implicit
    {
      return QoSInitialization(RMWQoSHistoryPolicy::KEEP_LAST, depth_);
    }
    size_t depth_;
  };

  struct KeepAll
  {
    KeepAll() = default;
    operator QoSInitialization() const                     // NOLINT implicit
    {
      return QoSInitialization(RMWQoSHistoryPolicy::KEEP_ALL, 0);
    }
  };

} // namespace lwrcl

#endif // LWRCL_QOS_HPP
