#ifndef LWRCL_CHANNEL_HPP_
#define LWRCL_CHANNEL_HPP_

#include <condition_variable>
#include <mutex>
#include <queue>

namespace lwrcl
{

  class ChannelCallback
  {
  public:
    virtual ~ChannelCallback() = default;
    virtual void invoke() = 0;
  };

  template <class T>
  class Channel
  {
  public:
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
      cv_.wait(lock, [this]
               { return !queue_.empty() || closed_; });
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
    bool closed_ = false;
    std::mutex mtx_;
    std::condition_variable cv_;
  };
} // namespace lwrcl

#endif // LWRCL_CHANNEL_HPP_
