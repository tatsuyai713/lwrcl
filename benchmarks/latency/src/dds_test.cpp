// Minimal test: DDS data flow through ara::com Adaptive AUTOSAR layer
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cstdio>
#include <atomic>
using namespace std::chrono_literals;

static std::atomic<int> recv_count{0};
static std::atomic<int> send_count{0};

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("dds_test") {
        printf("[init] Creating publisher on ping_req...\n"); fflush(stdout);
        pub_ = create_publisher<std_msgs::msg::String>("ping_req", rclcpp::QoS(1));
        printf("[init] Publisher created.\n"); fflush(stdout);

        printf("[init] Creating subscription on ping_req...\n"); fflush(stdout);
        sub_ = create_subscription<std_msgs::msg::String>(
            "ping_req", rclcpp::QoS(1),
            [](std::shared_ptr<std_msgs::msg::String> msg){
                recv_count.fetch_add(1);
                if (recv_count.load() % 100 == 1) {
                    printf("[sub] received msg #%d: %s\n", recv_count.load(), msg->data().substr(0, 40).c_str());
                    fflush(stdout);
                }
            });
        printf("[init] Subscription created.\n"); fflush(stdout);

        printf("[init] Creating timer (10ms)...\n"); fflush(stdout);
        timer_ = create_wall_timer(10ms, [this](){
            auto msg = std_msgs::msg::String();
            msg.data() = "test_" + std::to_string(send_count.fetch_add(1));
            pub_->publish(msg);
            if (send_count.load() % 100 == 1) {
                printf("[timer] sent #%d, recv=%d\n", send_count.load(), recv_count.load());
                fflush(stdout);
            }
            if (recv_count.load() >= 500) {
                printf("[done] sent=%d recv=%d\n", send_count.load(), recv_count.load());
                fflush(stdout);
                rclcpp::shutdown();
            }
        });
        printf("[init] Timer created. Entering spin...\n"); fflush(stdout);
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    printf("[main] init\n"); fflush(stdout);
    rclcpp::init(argc, argv);
    printf("[main] spinning\n"); fflush(stdout);
    rclcpp::spin(std::make_shared<TestNode>());
    printf("[main] done: sent=%d recv=%d\n", send_count.load(), recv_count.load());
    rclcpp::shutdown();
    return 0;
}
