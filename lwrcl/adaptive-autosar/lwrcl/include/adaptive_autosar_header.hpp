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
  // The communication endpoint is resolved by the Adaptive AUTOSAR runtime.
  // This wrapper keeps only domain_id as node-level context.
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
