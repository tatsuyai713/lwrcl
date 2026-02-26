// Minimal test: DDS ping-pong with 1ms timer (like benchmark)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cstdio>
#include <atomic>
using namespace std::chrono_literals;

static std::atomic<int> ping_sent{0};
static std::atomic<int> pong_recv{0};
static std::atomic<int> warmup_count{0};

class PingPongNode : public rclcpp::Node {
public:
    PingPongNode() : Node("ping_pong_1ms") {
        ping_pub_ = create_publisher<std_msgs::msg::String>("ping_req", rclcpp::QoS(1));
        pong_pub_ = create_publisher<std_msgs::msg::String>("ping_resp", rclcpp::QoS(1));

        ping_sub_ = create_subscription<std_msgs::msg::String>(
            "ping_req", rclcpp::QoS(1),
            [this](std::shared_ptr<std_msgs::msg::String> msg){
                pong_pub_->publish(*msg);
            });

        pong_sub_ = create_subscription<std_msgs::msg::String>(
            "ping_resp", rclcpp::QoS(1),
            [this](std::shared_ptr<std_msgs::msg::String> msg){
                (void)msg;
                int w = warmup_count.fetch_add(1);
                if (w < 2000) return;  // warmup
                pong_recv.fetch_add(1);
                if (pong_recv.load() >= 10000) rclcpp::shutdown();
            });

        // Same as benchmark: 1ms timer
        timer_ = create_wall_timer(1ms, [this](){
            if (pong_recv.load() >= 10000) return;
            auto msg = std_msgs::msg::String();
            msg.data() = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
            ping_pub_->publish(msg);
            int s = ping_sent.fetch_add(1);
            if (s % 1000 == 0) {
                printf("[status] sent=%d pong_recv=%d warmup=%d\n", s, pong_recv.load(), warmup_count.load());
                fflush(stdout);
            }
        });
        printf("[init] All created (1ms timer, 2000 warmup, 10000 samples)\n"); fflush(stdout);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_pub_, pong_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ping_sub_, pong_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingPongNode>());
    printf("[main] done: sent=%d pong_recv=%d warmup=%d\n",
        ping_sent.load(), pong_recv.load(), warmup_count.load());
    rclcpp::shutdown();
    return 0;
}
