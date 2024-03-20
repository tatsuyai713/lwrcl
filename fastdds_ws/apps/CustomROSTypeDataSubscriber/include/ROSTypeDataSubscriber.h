// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file ROSTypeDataSubscriber.h
 *
 */

#ifndef ROSTYPEDATASUBSCRIBER_H_
#define ROSTYPEDATASUBSCRIBER_H_

#include "CustomMessagePubSubTypes.h"
#include "CustomMessage.h"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/topic/Topic.hpp>

class ROSTypeDataSubscriber
{
public:

    ROSTypeDataSubscriber();

    virtual ~ROSTypeDataSubscriber();

    //!Initialize the subscriber
    bool init(std::string config_file_path);

    //!RUN the subscriber
    void run();


private:

    eprosima::fastdds::dds::DomainParticipant* participant_;
    eprosima::fastdds::dds::Subscriber* subscriber_;
    eprosima::fastdds::dds::Topic* topic_;
    eprosima::fastdds::dds::DataReader* reader_;
    eprosima::fastdds::dds::TypeSupport type_;
    std::string topic_name_;
    uint8_t domain_number_;


    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        SubListener()
            : matched_(0)
        {
            subscribe_msg_ = std::make_shared<CustomMessage>();
        }

        ~SubListener() override
        {
        }

        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override;

        void on_subscription_matched(
                eprosima::fastdds::dds::DataReader* reader,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

        std::shared_ptr<CustomMessage> subscribe_msg_;
        int matched_;
    }
    listener_;
};

#endif /* ROSTYPEDATASUBSCRIBER_H_ */
