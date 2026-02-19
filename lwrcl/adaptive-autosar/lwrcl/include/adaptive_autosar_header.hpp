#ifndef LWRCL_ADAPTIVE_AUTOSAR_HEADER_HPP_
#define LWRCL_ADAPTIVE_AUTOSAR_HEADER_HPP_

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

namespace lwrcl
{
  // -----------------------------------------------------------------------
  // Lightweight DomainParticipant wrapper for Adaptive AUTOSAR backend.
  //
  // Unlike the raw CycloneDDS backend where a real DDS participant is
  // shared across all publishers/subscribers on a Node, the Adaptive
  // AUTOSAR ara::com::dds layer creates its own DomainParticipant
  // internally for each publisher/subscriber.  This wrapper simply
  // stores the domain_id that is forwarded to DdsPublisher/DdsSubscriber
  // constructors.
  // -----------------------------------------------------------------------
  class AutosarDomainParticipant
  {
  public:
    explicit AutosarDomainParticipant(uint32_t domain_id = 0)
        : domain_id_(domain_id) {}

    uint32_t domain_id() const { return domain_id_; }

  private:
    uint32_t domain_id_;
  };

  using DomainParticipant = AutosarDomainParticipant;

  // -----------------------------------------------------------------------
  // Compatibility types (kept for API parity with other backends)
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

#endif // LWRCL_ADAPTIVE_AUTOSAR_HEADER_HPP_
