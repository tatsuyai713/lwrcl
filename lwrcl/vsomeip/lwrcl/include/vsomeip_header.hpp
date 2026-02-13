#ifndef LWRCL_VSOMEIP_HEADER_HPP_
#define LWRCL_VSOMEIP_HEADER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <vsomeip/vsomeip.hpp>

namespace lwrcl
{
  // -----------------------------------------------------------------------
  // Topic-name → SOME/IP ID mapping
  // -----------------------------------------------------------------------
  // SOME/IP identifies services/events with 16-bit numeric IDs.
  // We hash the ROS-style topic name to produce deterministic IDs.
  // -----------------------------------------------------------------------
  namespace someip_id
  {
    // FNV-1a 32-bit hash collapsed to 16-bit
    inline uint16_t hash16(const std::string &s)
    {
      uint32_t h = 2166136261u;
      for (char c : s)
      {
        h ^= static_cast<uint32_t>(static_cast<unsigned char>(c));
        h *= 16777619u;
      }
      // Fold to 16-bit and clamp to valid range [0x1000, 0xFFFE]
      uint16_t lo = static_cast<uint16_t>(h & 0xFFFF);
      uint16_t hi = static_cast<uint16_t>((h >> 16) & 0xFFFF);
      uint16_t combined = lo ^ hi;
      return static_cast<uint16_t>(0x1000 + (combined % 0xEFFF));
    }

    inline vsomeip::service_t topic_to_service_id(const std::string &topic)
    {
      return hash16(topic);
    }

    inline vsomeip::instance_t default_instance_id()
    {
      return 0x0001;
    }

    inline vsomeip::event_t default_event_id()
    {
      return 0x8001; // Events must be >= 0x8000 in SOME/IP
    }

    inline vsomeip::eventgroup_t default_eventgroup_id()
    {
      return 0x0001;
    }

    inline vsomeip::method_t default_request_method_id()
    {
      return 0x0001;
    }

    inline vsomeip::method_t default_response_method_id()
    {
      return 0x0002;
    }
  } // namespace someip_id

  // -----------------------------------------------------------------------
  // vsomeip Application wrapper — replaces DomainParticipant concept
  // -----------------------------------------------------------------------
  // In SOME/IP, a vsomeip::application is the central entity that manages
  // service offering, requesting, and message routing. It is analogous
  // to the DDS DomainParticipant.
  // -----------------------------------------------------------------------
  using DomainParticipant = vsomeip::application;

  // -----------------------------------------------------------------------
  // Compatibility types (unused in vsomeip but kept for API parity)
  // -----------------------------------------------------------------------
  namespace dds
  {
    using DomainId_t = uint32_t;
  }

  namespace rtps
  {
    enum class MemoryManagementPolicy_t
    {
      PREALLOCATED_WITH_REALLOC_MEMORY_MODE
    };

    enum class DiscoveryProtocol_t
    {
      SIMPLE,
      EXTERNAL
    };

    static const MemoryManagementPolicy_t PREALLOCATED_WITH_REALLOC_MEMORY_MODE =
        MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  } // namespace rtps

  class MessageType : public std::enable_shared_from_this<MessageType>
  {
  public:
    using SharedPtr = std::shared_ptr<MessageType>;
    MessageType() = default;

    template <typename T>
    explicit MessageType(T *message_type)
        : type_name_(typeid(T).name()) {}

    MessageType(const MessageType &other) = delete;
    MessageType &operator=(const MessageType &other) = delete;

    MessageType(MessageType &&other) noexcept = default;
    MessageType &operator=(MessageType &&other) noexcept = default;

    ~MessageType() = default;

    std::string get_type_name() const
    {
      return type_name_;
    }

  private:
    std::string type_name_;
  };

} // namespace lwrcl

template <typename T>
struct ParentTypeTraits;

#endif // LWRCL_VSOMEIP_HEADER_HPP_
