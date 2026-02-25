// lwrcl latency benchmark: intra-process ping-pong
// Single node with ping publisher/subscriber AND pong publisher/subscriber.
// Measures one-way latency = round-trip / 2.
//
// Build: see benchmarks/latency/build_lwrcl.sh
// Run:   see benchmarks/latency/run_all.sh
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cstdio>
using namespace std::chrono;
using namespace std::chrono_literals;

static std::vector<double> latencies;
static constexpr int WARMUP = 200;
static constexpr int MAX_SAMPLES = 1000;

class LatencyNode : public rclcpp::Node {
public:
    LatencyNode() : Node("latency"), warmup_(0) {
        // Ping publisher: sends timestamp
        ping_pub_ = create_publisher<std_msgs::msg::String>("ping_req", rclcpp::QoS(1));
        // Pong publisher: bounces ping back
        pong_pub_ = create_publisher<std_msgs::msg::String>("ping_resp", rclcpp::QoS(1));

        // Pong side: receive ping and bounce back
        ping_sub_ = create_subscription<std_msgs::msg::String>(
            "ping_req", rclcpp::QoS(1),
            [this](std::shared_ptr<std_msgs::msg::String> msg){
                pong_pub_->publish(*msg);
            });

        // Ping side: receive pong, measure latency
        pong_sub_ = create_subscription<std_msgs::msg::String>(
            "ping_resp", rclcpp::QoS(1),
            [this](std::shared_ptr<std_msgs::msg::String> msg){
                auto t2 = steady_clock::now().time_since_epoch().count();
                auto t1 = std::stoll(msg->data());
                double us = (t2 - t1) / 1000.0;
                if (warmup_++ < WARMUP) return;
                latencies.push_back(us);
                if ((int)latencies.size() >= MAX_SAMPLES) rclcpp::shutdown();
            });

        // Timer: send ping every 1 ms
        timer_ = create_wall_timer(1ms, [this](){
            if ((int)latencies.size() >= MAX_SAMPLES) return;
            auto msg = std_msgs::msg::String();
            msg.data() = std::to_string(steady_clock::now().time_since_epoch().count());
            ping_pub_->publish(msg);
        });
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_pub_, pong_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ping_sub_, pong_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int warmup_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LatencyNode>());
    rclcpp::shutdown();

    std::sort(latencies.begin(), latencies.end());
    int n = latencies.size();
    if (n < 10) { printf("Only %d samples\n", n); return 1; }
    double mean = std::accumulate(latencies.begin(), latencies.end(), 0.0) / n;
    printf("Samples: %d\n", n);
    printf("Mean:  %.1f us\n", mean);
    printf("p50:   %.1f us\n", latencies[n*50/100]);
    printf("p90:   %.1f us\n", latencies[n*90/100]);
    printf("p99:   %.1f us\n", latencies[n*99/100]);
    printf("Min:   %.1f us\n", latencies[0]);
    return 0;
}
