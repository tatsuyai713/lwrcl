// Minimal test: DDS ping-pong through ara::com (2 topics, like benchmark)
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cstdio>
#include <atomic>
using namespace std::chrono_literals;

static std::atomic<int> ping_sent{0};
static std::atomic<int> ping_recv{0};
static std::atomic<int> pong_recv{0};

class PingPongNode : public rclcpp::Node {
public:
    PingPongNode() : Node("ping_pong") {
        printf("[init] Creating ping publisher (ping_req)...\n"); fflush(stdout);
        ping_pub_ = create_publisher<std_msgs::msg::String>("ping_req", rclcpp::QoS(1));
        printf("[init] Creating pong publisher (ping_resp)...\n"); fflush(stdout);
        pong_pub_ = create_publisher<std_msgs::msg::String>("ping_resp", rclcpp::QoS(1));

        printf("[init] Creating ping subscriber (ping_req)...\n"); fflush(stdout);
        ping_sub_ = create_subscription<std_msgs::msg::String>(
            "ping_req", rclcpp::QoS(1),
            [this](std::shared_ptr<std_msgs::msg::String> msg){
                ping_recv.fetch_add(1);
                pong_pub_->publish(*msg);
            });

        printf("[init] Creating pong subscriber (ping_resp)...\n"); fflush(stdout);
        pong_sub_ = create_subscription<std_msgs::msg::String>(
            "ping_resp", rclcpp::QoS(1),
            [this](std::shared_ptr<std_msgs::msg::String> msg){
                pong_recv.fetch_add(1);
                (void)msg;
            });

        printf("[init] Creating timer (10ms)...\n"); fflush(stdout);
        timer_ = create_wall_timer(10ms, [this](){
            auto msg = std_msgs::msg::String();
            msg.data() = "ping_" + std::to_string(ping_sent.fetch_add(1));
            ping_pub_->publish(msg);
            if (ping_sent.load() % 50 == 1) {
                printf("[status] sent=%d ping_recv=%d pong_recv=%d\n",
                    ping_sent.load(), ping_recv.load(), pong_recv.load());
                fflush(stdout);
            }
            if (pong_recv.load() >= 500) {
                printf("[done] sent=%d ping_recv=%d pong_recv=%d\n",
                    ping_sent.load(), ping_recv.load(), pong_recv.load());
                fflush(stdout);
                rclcpp::shutdown();
            }
        });
        printf("[init] All created. Entering spin...\n"); fflush(stdout);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_pub_, pong_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ping_sub_, pong_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingPongNode>());
    printf("[main] done: sent=%d ping_recv=%d pong_recv=%d\n",
        ping_sent.load(), ping_recv.load(), pong_recv.load());
    rclcpp::shutdown();
    return 0;
}
