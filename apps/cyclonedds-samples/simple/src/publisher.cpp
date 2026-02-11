#include <dds/dds.hpp>
#include <std_msgs/msg/Header.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    dds::domain::DomainParticipant participant(0);
    dds::topic::Topic<std_msgs::msg::Header> topic(participant, "rt/HeaderTopic");
    dds::pub::Publisher publisher(participant);
    dds::pub::qos::DataWriterQos qos = publisher.default_datawriter_qos()
          << dds::core::policy::Reliability::Reliable()
          << dds::core::policy::Durability::Volatile()
          << dds::core::policy::History::KeepLast(10);
    dds::pub::DataWriter<std_msgs::msg::Header> writer(publisher, topic, qos);

    std_msgs::msg::Header header;
    header.stamp().sec() = 1627543200;
    header.stamp().nanosec() = 123456789;
    header.frame_id() = "frame_1";

    while (true) {
        writer.write(header);
        std::cout << "Published: " << header.frame_id() << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}