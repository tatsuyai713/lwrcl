#ifndef LWRCL_SERIALIZED_MESSAGE_HPP_
#define LWRCL_SERIALIZED_MESSAGE_HPP_

#include <cstddef>
#include <cstring>

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
    SerializedMessage() : data_(), is_own_buffer_(true)
    {
      data_.buffer = nullptr;
      data_.length = 0;
      data_.capacity = 0;
    }

    explicit SerializedMessage(size_t initial_capacity) : data_(), is_own_buffer_(true)
    {
      data_.buffer = new char[initial_capacity];
      data_.length = 0;
      data_.capacity = initial_capacity;
    }

    SerializedMessage(const SerializedMessage &other) : data_(), is_own_buffer_(true)
    {
      if (other.data_.length > 0 && other.data_.buffer)
      {
        data_.buffer = new char[other.data_.length];
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

    SerializedMessage(const lwrcl_serialized_message_t &other) : data_(), is_own_buffer_(true)
    {
      if (other.length > 0 && other.buffer)
      {
        data_.buffer = new char[other.length];
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
        : data_(other.data_), is_own_buffer_(other.is_own_buffer_)
    {
      other.data_.buffer = nullptr;
      other.data_.length = 0;
      other.data_.capacity = 0;
    }

    SerializedMessage(lwrcl_serialized_message_t &&other) noexcept : data_(other), is_own_buffer_(true)
    {
      other.buffer = nullptr;
      other.length = 0;
      other.capacity = 0;
    }

    ~SerializedMessage()
    {
      if (data_.buffer != nullptr && is_own_buffer_)
      {
        delete[] data_.buffer;
      }
    }

    SerializedMessage &operator=(const SerializedMessage &other)
    {
      if (this != &other)
      {
        if (is_own_buffer_ && data_.capacity >= other.data_.length && data_.buffer != nullptr)
        {
          std::memcpy(data_.buffer, other.data_.buffer, other.data_.length);
          data_.length = other.data_.length;
        }
        else
        {
          if (data_.buffer != nullptr && is_own_buffer_)
          {
            delete[] data_.buffer;
          }
          data_.buffer = new char[other.data_.length];
          std::memcpy(data_.buffer, other.data_.buffer, other.data_.length);
          data_.length = other.data_.length;
          data_.capacity = other.data_.length;
          is_own_buffer_ = true;
        }
      }
      return *this;
    }

    SerializedMessage &operator=(const lwrcl_serialized_message_t &other)
    {
      if (data_.buffer != other.buffer)
      {
        if (is_own_buffer_ && data_.capacity >= other.length && data_.buffer != nullptr)
        {
          std::memcpy(data_.buffer, other.buffer, other.length);
          data_.length = other.length;
        }
        else
        {
          if (data_.buffer != nullptr && is_own_buffer_)
          {
            delete[] data_.buffer;
          }
          data_.buffer = new char[other.length];
          std::memcpy(data_.buffer, other.buffer, other.length);
          data_.length = other.length;
          data_.capacity = other.length;
          is_own_buffer_ = true;
        }
      }
      return *this;
    }

    SerializedMessage &operator=(SerializedMessage &&other) noexcept
    {
      if (this != &other)
      {
        if (data_.buffer != nullptr && is_own_buffer_)
        {
          delete[] data_.buffer;
        }
        data_ = other.data_;
        is_own_buffer_ = other.is_own_buffer_;
        other.data_.buffer = nullptr;
        other.data_.length = 0;
        other.data_.capacity = 0;
      }
      return *this;
    }

    SerializedMessage &operator=(lwrcl_serialized_message_t &&other) noexcept
    {
      if (data_.buffer != other.buffer)
      {
        if (data_.buffer != nullptr && is_own_buffer_)
        {
          delete[] data_.buffer;
        }
        data_ = other;
        other.buffer = nullptr;
        other.length = 0;
        other.capacity = 0;
        is_own_buffer_ = true;
      }
      return *this;
    }

    lwrcl_serialized_message_t &get_rcl_serialized_message() { return data_; }
    const lwrcl_serialized_message_t &get_rcl_serialized_message() const { return data_; }

    void set_buffer(char *buffer, size_t length)
    {
      if (data_.buffer != nullptr && is_own_buffer_)
      {
        delete[] data_.buffer;
      }
      data_.buffer = buffer;
      data_.length = length;
      data_.capacity = length;
      is_own_buffer_ = false;
    }

    size_t size() const { return data_.length; }
    size_t capacity() const { return data_.capacity; }

    void reserve(size_t new_capacity)
    {
      if (new_capacity > data_.capacity)
      {
        char *new_buffer = new char[new_capacity];
        if (data_.buffer != nullptr && data_.length > 0)
        {
          std::memcpy(new_buffer, data_.buffer, data_.length);
        }
        if (data_.buffer != nullptr && is_own_buffer_)
        {
          delete[] data_.buffer;
        }
        data_.buffer = new_buffer;
        data_.capacity = new_capacity;
        is_own_buffer_ = true;
      }
    }

    lwrcl_serialized_message_t release_lwrcl_serialized_message()
    {
      lwrcl_serialized_message_t out = data_;
      data_.buffer = nullptr;
      data_.length = 0;
      data_.capacity = 0;
      is_own_buffer_ = false;
      return out;
    }

  private:
    lwrcl_serialized_message_t data_;
    bool is_own_buffer_;
  };

} // namespace lwrcl

#endif // LWRCL_SERIALIZED_MESSAGE_HPP_
