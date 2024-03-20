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
 * @file ROSTypeDataSubscriber.cpp
 *
 */

#include "ROSTypeDataSubscriber.h"

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include <thread>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
// #include <sys/timerfd.h>
#include <time.h>
#include <yaml-cpp/yaml.h>

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

volatile sig_atomic_t stop_flag = 0;
void abort_handler(int sig);

void abort_handler(int sig){
	stop_flag = 1;
}

ROSTypeDataSubscriber::ROSTypeDataSubscriber()
    : participant_(nullptr)
    , subscriber_(nullptr)
    , topic_(nullptr)
    , reader_(nullptr)
    , type_(new CustomMessagePubSubType())
    , domain_number_(0)
    , topic_name_(std::string("default_topic"))
{
}

bool ROSTypeDataSubscriber::init(std::string config_file_path)
{
    YAML::Node node = YAML::LoadFile(config_file_path);
    YAML::Node config = node["config"];

    bool shared_memory = config["shared_memory"].as<bool>(false);
    bool ros2_topic = config["ros2_topic"].as<bool>(true);
    domain_number_ = config["domain_number"].as<uint8_t>(0);
    std::cout << "shared_memory = " << shared_memory << std::endl;
    std::cout << "ros2_topic = " << ros2_topic << std::endl;
    std::cout << "domain_number = " << int(domain_number_) << std::endl;

    if(ros2_topic == true && shared_memory == true) 
    {
        std::cout << "ROS 2 compatible topic doesn't support Shared Memory!" << std::endl;
        return false;
    }

    YAML::Node topics = config["topics"];
    if (topics.IsSequence()) {
        for (const auto topic_node: topics) {
            topic_name_ = topic_node["name"].as<std::string>("default_topic");
            if(ros2_topic)
            {
                topic_name_ = "rt/" + topic_name_;
            }
        }
    }

    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    TopicQos tqos = TOPIC_QOS_DEFAULT;
    SubscriberQos subqos = SUBSCRIBER_QOS_DEFAULT;
    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;

    if(shared_memory)
    {
        //CREATE THE PARTICIPANT
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SIMPLE;
        pqos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
        pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
        pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
        pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
        pqos.name("Participant_sub");

        // Explicit configuration of SharedMem transport
        pqos.transport().use_builtin_transports = false;

        auto shm_transport = std::make_shared<SharedMemTransportDescriptor>();
        shm_transport->segment_size(2 * 1024 * 1024);
        pqos.transport().user_transports.push_back(shm_transport);

        participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_number_, pqos);

        if (participant_ == nullptr)
        {
            return false;
        }

        //REGISTER THE TYPE
        type_.register_type(participant_);

        //CREATE THE SUBSCRIBER
        subscriber_ = participant_->create_subscriber(subqos);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        //CREATE THE TOPIC
        topic_ = participant_->create_topic(topic_name_, type_->getName(), tqos);

        if (topic_ == nullptr)
        {
            return false;
        }

        //CREATE THE DATAREADER
        rqos.history().kind = KEEP_LAST_HISTORY_QOS;
        rqos.history().depth = 30;
        rqos.resource_limits().max_samples = 50;
        rqos.resource_limits().allocated_samples = 20;
        rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        rqos.durability().kind = TRANSIENT_LOCAL_DURABILITY_QOS;

        reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }
    }
    else
    {
        pqos.name("Participant_sub");
        auto factory = DomainParticipantFactory::get_instance();

        participant_ = factory->create_participant(domain_number_, pqos);

        if (participant_ == nullptr)
        {
            return false;
        }

        //REGISTER THE TYPE
        type_.register_type(participant_);

        //CREATE THE SUBSCRIBER
        subscriber_ = participant_->create_subscriber(subqos, nullptr);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        //CREATE THE TOPIC
        topic_ = participant_->create_topic(topic_name_, type_->getName(), tqos);

        if (topic_ == nullptr)
        {
            return false;
        }

        //CREATE THE READER
        rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;

        rqos.endpoint().history_memory_policy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

        reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }
    }

    return true;
}

ROSTypeDataSubscriber::~ROSTypeDataSubscriber()
{
    if (reader_ != nullptr)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

void ROSTypeDataSubscriber::SubListener::on_subscription_matched(
        eprosima::fastdds::dds::DataReader *,
     const eprosima::fastdds::dds::SubscriptionMatchedStatus &info)
{
    if (info.current_count_change == 1)
    {
        matched_ = info.total_count;
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched_ = info.total_count;
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}

void ROSTypeDataSubscriber::SubListener::on_data_available(
        eprosima::fastdds::dds::DataReader *reader)
{
    SampleInfo info;
    if (reader->take_next_sample(subscribe_msg_.get(), &info) == ReturnCode_t::RETCODE_OK)
    {
        if (info.instance_state == ALIVE_INSTANCE_STATE)
        {
            // Print your structure data here.
            std::cout << "Message " << subscribe_msg_->message() << " " << subscribe_msg_->index() << " RECEIVED" << std::endl;

        }
    }
}

void ROSTypeDataSubscriber::run()
{
    std::cout << "Subscriber running." << std::endl;

	if (signal(SIGINT,abort_handler) == SIG_ERR){
		printf("Singal Handler set error!!\n");
		exit(1);
	}

    while(!stop_flag)
    {
        usleep(1000);
    }
}
