// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file CyclicROSDataPublisher.h
 *
 */

#ifndef CyclicROSDATAPUBLISHER_H_
#define CyclicROSDATAPUBLISHER_H_

#include "geometry_msgs/msg/PosePubSubTypes.h"
#include "geometry_msgs/msg/Pose.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/Topic.hpp>

class CyclicROSDataPublisher
{
public:

    CyclicROSDataPublisher();

    virtual ~CyclicROSDataPublisher();

    //!Initialize
    bool init(std::string config_file_path);

    //!Publish
    bool publish(bool waitForListener = true);

    //!Run for number samples
    void run();

private:

    std::shared_ptr<geometry_msgs::msg::Pose> publish_msg_;
    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Publisher* publisher_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataWriter* writer_;
    eprosima::fastdds::dds::TypeSupport type_;
    std::string topic_name_;
    uint16_t interval_ms_;
    uint8_t domain_number_;

    class PubListener : public eprosima::fastdds::dds::DataWriterListener
    {
    public:
        PubListener()
            : matched_(0)
            , first_connected_(false)
        {
        }
        ~PubListener() override
        {
        }
        void on_publication_matched(
                eprosima::fastdds::dds::DataWriter* writer,
                const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

        int matched_;
        bool first_connected_;

    } listener_;

    void runThread();
};

#endif /* CyclicROSDATAPUBLISHER_H_ */
