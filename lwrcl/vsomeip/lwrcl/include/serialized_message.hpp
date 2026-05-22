#ifndef LWRCL_SERIALIZED_MESSAGE_HPP_
#define LWRCL_SERIALIZED_MESSAGE_HPP_

#include <cstddef>
#include <cstring>
#include <memory>

namespace lwrcl
{

  struct lwrcl_serialized_message_t
  {
    char *buffer;
    size_t length;
    size_t capacity;
  };

  class SerializedMessage
  {
  public:
    SerializedMessage() : data_(), owned_buffer_()
    {
      data_.buffer = nullptr;
      data_.length = 0;
      data_.capacity = 0;
    }

    explicit SerializedMessage(size_t initial_capacity) : data_(), owned_buffer_()
    {
      owned_buffer_.reset(new char[initial_capacity]);
      data_.buffer = owned_buffer_.get();
      data_.length = 0;
      data_.capacity = initial_capacity;
    }

    SerializedMessage(const SerializedMessage &other) : data_(), owned_buffer_()
    {
      if (other.data_.length > 0 && other.data_.buffer)
      {
        owned_buffer_.reset(new char[other.data_.length]);
        data_.buffer = owned_buffer_.get();
        std::memcpy(data_.buffer, other.data_.buffer, other.data_.length);
        data_.length = other.data_.length;
        data_.capacity = other.data_.length;
      }
      else
      {
        data_.buffer = nullptr;
        data_.length = 0;
        data_.capacity = 0;
      }
    }

    SerializedMessage(const lwrcl_serialized_message_t &other) : data_(), owned_buffer_()
    {
      if (other.length > 0 && other.buffer)
      {
        owned_buffer_.reset(new char[other.length]);
        data_.buffer = owned_buffer_.get();
        std::memcpy(data_.buffer, other.buffer, other.length);
        data_.length = other.length;
        data_.capacity = other.length;
      }
      else
      {
        data_.buffer = nullptr;
        data_.length = 0;
        data_.capacity = 0;
      }
    }

    SerializedMessage(SerializedMessage &&other) noexcept
        : data_(other.data_), owned_buffer_(std::move(other.owned_buffer_))
    {
      if (owned_buffer_)
      {
        data_.buffer = owned_buffer_.get();
      }
      other.data_.buffer = nullptr;
      other.data_.length = 0;
      other.data_.capacity = 0;
    }

    SerializedMessage(lwrcl_serialized_message_t &&other) noexcept : data_(), owned_buffer_()
    {
      if (other.buffer == nullptr || other.capacity == 0)
      {
        delete[] other.buffer;
        data_.buffer = nullptr;
        data_.length = 0;
        data_.capacity = 0;
        other.buffer = nullptr;
        other.length = 0;
        other.capacity = 0;
        return;
      }
      owned_buffer_.reset(other.buffer);
      data_.buffer = owned_buffer_.get();
      data_.length = other.length;
      data_.capacity = other.capacity;
      other.buffer = nullptr;
      other.length = 0;
      other.capacity = 0;
    }

    ~SerializedMessage() = default;

    SerializedMessage &operator=(const SerializedMessage &other)
    {
      if (this != &other)
      {
        if (other.data_.length == 0 || other.data_.buffer == nullptr)
        {
          owned_buffer_.reset();
          data_.buffer = nullptr;
          data_.length = 0;
          data_.capacity = 0;
          return *this;
        }
        if (owned_buffer_ && data_.capacity >= other.data_.length)
        {
          std::memcpy(data_.buffer, other.data_.buffer, other.data_.length);
          data_.length = other.data_.length;
        }
        else
        {
          owned_buffer_.reset(other.data_.length > 0 ? new char[other.data_.length] : nullptr);
          data_.buffer = owned_buffer_.get();
          std::memcpy(data_.buffer, other.data_.buffer, other.data_.length);
          data_.length = other.data_.length;
          data_.capacity = other.data_.length;
        }
      }
      return *this;
    }

    SerializedMessage &operator=(const lwrcl_serialized_message_t &other)
    {
      if (other.length == 0 || other.buffer == nullptr)
      {
        owned_buffer_.reset();
        data_.buffer = nullptr;
        data_.length = 0;
        data_.capacity = 0;
        return *this;
      }
      if (data_.buffer != other.buffer)
      {
        if (owned_buffer_ && data_.capacity >= other.length)
        {
          std::memcpy(data_.buffer, other.buffer, other.length);
          data_.length = other.length;
        }
        else
        {
          owned_buffer_.reset(other.length > 0 ? new char[other.length] : nullptr);
          data_.buffer = owned_buffer_.get();
          std::memcpy(data_.buffer, other.buffer, other.length);
          data_.length = other.length;
          data_.capacity = other.length;
        }
      }
      return *this;
    }

    SerializedMessage &operator=(SerializedMessage &&other) noexcept
    {
      if (this != &other)
      {
        data_ = other.data_;
        owned_buffer_ = std::move(other.owned_buffer_);
        if (owned_buffer_)
        {
          data_.buffer = owned_buffer_.get();
        }
        other.data_.buffer = nullptr;
        other.data_.length = 0;
        other.data_.capacity = 0;
      }
      return *this;
    }

    SerializedMessage &operator=(lwrcl_serialized_message_t &&other) noexcept
    {
      if (&other == &data_)
      {
        return *this;
      }
      if (other.buffer == nullptr || other.capacity == 0)
      {
        if (owned_buffer_.get() == other.buffer)
        {
          owned_buffer_.reset();
        }
        else
        {
          owned_buffer_.reset();
          delete[] other.buffer;
        }
        data_.buffer = nullptr;
        data_.length = 0;
        data_.capacity = 0;
        other.buffer = nullptr;
        other.length = 0;
        other.capacity = 0;
        return *this;
      }
      if (owned_buffer_.get() != other.buffer)
      {
        owned_buffer_.reset(other.buffer);
      }
      data_.buffer = owned_buffer_.get();
      data_.length = other.length;
      data_.capacity = other.capacity;
      other.buffer = nullptr;
      other.length = 0;
      other.capacity = 0;
      return *this;
    }

    lwrcl_serialized_message_t &get_rcl_serialized_message() { return data_; }
    const lwrcl_serialized_message_t &get_rcl_serialized_message() const { return data_; }

    void set_buffer(char *buffer, size_t length)
    {
      owned_buffer_.reset();
      data_.buffer = buffer;
      data_.length = length;
      data_.capacity = length;
    }

    size_t size() const { return data_.length; }
    size_t capacity() const { return data_.capacity; }

    void reserve(size_t new_capacity)
    {
      if (new_capacity > data_.capacity)
      {
        std::unique_ptr<char[]> new_buffer(new char[new_capacity]);
        if (data_.buffer != nullptr && data_.length > 0)
        {
          std::memcpy(new_buffer.get(), data_.buffer, data_.length);
        }
        owned_buffer_ = std::move(new_buffer);
        data_.buffer = owned_buffer_.get();
        data_.capacity = new_capacity;
      }
    }

    lwrcl_serialized_message_t release_lwrcl_serialized_message()
    {
      lwrcl_serialized_message_t out = data_;
      if (owned_buffer_)
      {
        out.buffer = owned_buffer_.release();
      }
      data_.buffer = nullptr;
      data_.length = 0;
      data_.capacity = 0;
      return out;
    }

  private:
    lwrcl_serialized_message_t data_;
    std::unique_ptr<char[]> owned_buffer_;
  };

} // namespace lwrcl

#endif // LWRCL_SERIALIZED_MESSAGE_HPP_
