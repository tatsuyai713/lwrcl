#ifndef LWRCL_CHANNEL_HPP_
#define LWRCL_CHANNEL_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <utility>

namespace lwrcl
{

  class ChannelCallback
  {
  public:
    virtual ~ChannelCallback() = default;
    virtual void invoke() = 0;

    ChannelCallback(const ChannelCallback &) = delete;
    ChannelCallback &operator=(const ChannelCallback &) = delete;
    ChannelCallback(ChannelCallback &&) = default;
    ChannelCallback &operator=(ChannelCallback &&) = default;

  protected:
    ChannelCallback() = default;
  };

  template <class T>
  class Channel : public std::enable_shared_from_this<Channel<T>>
  {
  public:
    using SharedPtr = std::shared_ptr<Channel<T>>;

    Channel() : queue_(), closed_(false), mtx_(), cv_() {}

    ~Channel() = default;

    Channel(const Channel &) = delete;
    Channel &operator=(const Channel &) = delete;
    Channel(Channel &&) = default;
    Channel &operator=(Channel &&) = default;

    void produce(T &&x)
    {
      std::lock_guard<std::mutex> lock{mtx_};
      if (!closed_)
      {
        queue_.push(std::forward<T>(x));
        cv_.notify_all();
      }
    }

    bool consume(T &x)
    {
      std::unique_lock<std::mutex> lock{mtx_};
      if (!cv_.wait_for(lock, std::chrono::milliseconds(100), [this]
                        { return !queue_.empty() || closed_; }))
      {
        return false;
      }
      if (closed_ && queue_.empty())
      {
        return false;
      }
      x = std::move(queue_.front());
      queue_.pop();
      return true;
    }

    bool consume_nowait(T &x)
    {
      std::lock_guard<std::mutex> lock{mtx_};

      if (queue_.empty())
      {
        return false;
      }
      x = std::move(queue_.front());
      queue_.pop();
      return true;
    }

    void close()
    {
      std::lock_guard<std::mutex> lock{mtx_};
      closed_ = true;
      cv_.notify_all();
    }

    bool is_closed()
    {
      std::lock_guard<std::mutex> lock{mtx_};
      return closed_;
    }

  private:
    std::queue<T> queue_;
    bool closed_;
    std::mutex mtx_;
    std::condition_variable cv_;
  };
} // namespace lwrcl

#endif // LWRCL_CHANNEL_HPP_
