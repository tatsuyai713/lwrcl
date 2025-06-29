#ifndef LWRCL_QOS_HPP
#define LWRCL_QOS_HPP

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

// Custom QoS profile structure
struct RMWQoSProfile
{
  RMWQoSProfile()
      : depth(10), history(RMWQoSHistoryPolicy::KEEP_LAST), reliability(RMWQoSReliabilityPolicy::RELIABLE), durability(RMWQoSDurabilityPolicy::VOLATILE)
  {
  }

  RMWQoSProfile(
      size_t depth,
      RMWQoSHistoryPolicy history,
      RMWQoSReliabilityPolicy reliability,
      RMWQoSDurabilityPolicy durability)
      : depth(depth), history(history), reliability(reliability), durability(durability)
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

    // Default constructor
    explicit QoS(size_t depth = 10)
        : depth_(depth),
          history_(HistoryPolicy::KEEP_LAST),
          reliability_(ReliabilityPolicy::RELIABLE),
          durability_(DurabilityPolicy::VOLATILE)
    {
    }

    // Constructor with HistoryPolicy and custom profile
    QoS(HistoryPolicy history, const RMWQoSProfile &custom_profile)
        : depth_(custom_profile.depth),
          history_(history),
          reliability_(custom_profile.reliability == RMWQoSReliabilityPolicy::BEST_EFFORT ? ReliabilityPolicy::BEST_EFFORT : ReliabilityPolicy::RELIABLE),
          durability_(custom_profile.durability == RMWQoSDurabilityPolicy::VOLATILE ? DurabilityPolicy::VOLATILE : DurabilityPolicy::TRANSIENT_LOCAL) {}

    QoS(const QoSInitialization qos, const RMWQoSProfile &custom_profile)
        : depth_(qos.depth_),
          history_(qos.history_ == RMWQoSHistoryPolicy::KEEP_LAST ? HistoryPolicy::KEEP_LAST : HistoryPolicy::KEEP_ALL),
          reliability_(custom_profile.reliability == RMWQoSReliabilityPolicy::BEST_EFFORT ? ReliabilityPolicy::BEST_EFFORT : ReliabilityPolicy::RELIABLE),
          durability_(custom_profile.durability == RMWQoSDurabilityPolicy::VOLATILE ? DurabilityPolicy::VOLATILE : DurabilityPolicy::TRANSIENT_LOCAL) {}

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

    // Set DurabilityPolicy
    QoS &durability(DurabilityPolicy policy)
    {
      durability_ = policy;
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

      return profile;
    }

  private:
    size_t depth_;
    HistoryPolicy history_;
    ReliabilityPolicy reliability_;
    DurabilityPolicy durability_;
  };

  extern const RMWQoSProfile rmw_qos_profile_default;
  extern const RMWQoSProfile rmw_qos_profile_sensor_data;
  extern const RMWQoSProfile rmw_qos_profile_parameters;
  extern const RMWQoSProfile rmw_qos_profile_services_default;
  extern const RMWQoSProfile rmw_qos_profile_parameter_events;

  QoSInitialization KeepLast(size_t depth);
  QoSInitialization KeepAll();
} // namespace lwrcl

#endif // LWRCL_QOS_HPP