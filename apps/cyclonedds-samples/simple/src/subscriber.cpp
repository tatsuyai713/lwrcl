#include <dds/dds.hpp>
#include <std_msgs/msg/Header.hpp>
#include <iostream>

int main()
{
    // DomainParticipantの作成
    dds::domain::DomainParticipant participant(0); // ドメインIDを0に設定

    // Topicの作成
    dds::topic::Topic<std_msgs::msg::Header> topic(participant, "rt/HeaderTopic");

    // Subscriberの作成
    dds::sub::Subscriber subscriber(participant);

    // QoSの設定

dds::sub::qos::DataReaderQos qos = subscriber.default_datareader_qos()
    << dds::core::policy::Reliability::Reliable()
    << dds::core::policy::Durability::Volatile()
    << dds::core::policy::History::KeepLast(10); // 最後の10件を保持

    // DataReaderの作成
    dds::sub::DataReader<std_msgs::msg::Header> reader(subscriber, topic, qos);

    // ReadConditionの作成
    dds::sub::cond::ReadCondition read_condition(reader, dds::sub::status::DataState::new_data());

    // WaitSetの作成
    dds::core::cond::WaitSet waitset;
    waitset += read_condition;

    // データ受信の待機
    while (true)
    {
        try
        {
            waitset.dispatch(dds::core::Duration::from_secs(10));
            dds::sub::LoanedSamples<std_msgs::msg::Header> samples = reader.take();
            for (const auto &sample : samples)
            {
                if (sample.info().valid())
                {
                    std_msgs::msg::Header header = sample.data();
                    std::cout << "Received: " << header.frame_id() << std::endl;
                }
            }
        }
        catch (const dds::core::TimeoutError &e)
        {
            // タイムアウト時の処理
            std::cerr << "Timeout occurred: " << e.what() << std::endl;
        }
    }

    return 0;
}