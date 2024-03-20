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
 * @file ROSTypeDataPublisher.cpp
 *
 */

#include "ROSTypeDataPublisher.h"

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
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

volatile sig_atomic_t stop_flag = 0;
pthread_mutex_t m = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

void abort_handler(int sig);
void timer_func(union sigval arg);

void abort_handler(int sig){
	stop_flag = 1;
}

void timer_func(union sigval arg){
	printf("Tid = %lu\n",pthread_self());
	pthread_cond_signal(&cond);
}

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

ROSTypeDataPublisher::ROSTypeDataPublisher()
    : participant_(nullptr)
    , publisher_(nullptr)
    , topic_(nullptr)
    , writer_(nullptr)
    , type_(new CustomMessagePubSubType())
    , domain_number_(0)
    , topic_name_(std::string("default_topic"))
    , interval_ms_(1000)
{
}

bool ROSTypeDataPublisher::init(std::string config_file_path)
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
            interval_ms_ = topic_node["interval_ms"].as<uint16_t>(1000);
            std::cout << "name == " << topic_name_ << ", interval == " << interval_ms_ <<  "[ms]" << std::endl;
        }
    }

    if(interval_ms_ == 0) 
    {
        std::cout << "Interval Time Error!" << std::endl;
        return false;
    }

    publish_msg_ = std::make_shared<CustomMessage>();

    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    TopicQos tqos = TOPIC_QOS_DEFAULT;
    PublisherQos pubqos = PUBLISHER_QOS_DEFAULT;
    DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;

    if(shared_memory)
    {
        //CREATE THE PARTICIPANT
        pqos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SIMPLE;
        pqos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
        pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
        pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
        pqos.wire_protocol().builtin.discovery_config.leaseDuration = eprosima::fastrtps::c_TimeInfinite;
        pqos.name("Participant_pub");

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

        //CREATE THE PUBLISHER
        publisher_ = participant_->create_publisher(pubqos);

        if (publisher_ == nullptr)
        {
            return false;
        }

        //CREATE THE TOPIC
        topic_ = participant_->create_topic(topic_name_, type_->getName(), tqos);

        if (topic_ == nullptr)
        {
            return false;
        }

        //CREATE THE DATAWRITER
        wqos.history().kind = KEEP_LAST_HISTORY_QOS;
        wqos.history().depth = 30;
        wqos.resource_limits().max_samples = 50;
        wqos.resource_limits().allocated_samples = 20;
        wqos.reliable_writer_qos().times.heartbeatPeriod.seconds = 2;
        wqos.reliable_writer_qos().times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
        wqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        wqos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;

        writer_ = publisher_->create_datawriter(topic_, wqos, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }

    }
    else
    {
        pqos.name("Participant_pub");
        auto factory = DomainParticipantFactory::get_instance();

        participant_ = factory->create_participant(domain_number_, pqos);

        if (participant_ == nullptr)
        {
            return false;
        }

        //REGISTER THE TYPE
        type_.register_type(participant_);

        //CREATE THE PUBLISHER
        publisher_ = participant_->create_publisher(pubqos, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        //CREATE THE TOPIC
        topic_ = participant_->create_topic(topic_name_, type_->getName(), tqos);

        if (topic_ == nullptr)
        {
            return false;
        }

        // CREATE THE WRITER
        wqos.endpoint().history_memory_policy = eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

        writer_ = publisher_->create_datawriter(topic_, wqos, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
    }

    return true;
}

ROSTypeDataPublisher::~ROSTypeDataPublisher()
{
    if (writer_ != nullptr)
    {
        publisher_->delete_datawriter(writer_);
    }
    if (publisher_ != nullptr)
    {
        participant_->delete_publisher(publisher_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

void ROSTypeDataPublisher::PubListener::on_publication_matched(
        eprosima::fastdds::dds::DataWriter*,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched_ = info.total_count;
        first_connected_ = true;
        std::cout << "Publisher matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched_ = info.total_count;
        std::cout << "Publisher unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
    }
}

void ROSTypeDataPublisher::runThread()
{
	int ret;
	struct timespec curTime,lastTime;
	clock_gettime(CLOCK_REALTIME,&lastTime);

	pthread_mutex_lock(&m);
    while (!stop_flag){
		ret = pthread_cond_wait(&cond,&m);
		if(ret == 0){
			clock_gettime(CLOCK_REALTIME,&curTime);
			if(curTime.tv_nsec < lastTime.tv_nsec){
				printf("Interval = %10ld.%09ld\n",curTime.tv_sec - lastTime.tv_sec - 1,curTime.tv_nsec + 1000000000 - lastTime.tv_nsec);
			}
			else{
				printf("Interval = %10ld.%09ld\n",curTime.tv_sec - lastTime.tv_sec,curTime.tv_nsec - lastTime.tv_nsec);
			}
			lastTime = curTime;


            if (publish(false))
            {
                std::cout << "Message: " << publish_msg_->message() << " with index: " << publish_msg_->index() << " SENT" << std::endl;
            }
		}
    }
	pthread_mutex_unlock(&m);

}

void ROSTypeDataPublisher::run()
{
    timer_t timer_id;
	struct itimerspec ts;
	struct sigevent se;
	int status;
	int ret;

	if (signal(SIGINT,abort_handler) == SIG_ERR){
		printf("Singal Handler set error!!\n");
		exit(1);
	}
	se.sigev_notify = SIGEV_THREAD;
	se.sigev_value.sival_ptr = &timer_id;
	se.sigev_notify_function = timer_func;
	se.sigev_notify_attributes = NULL;

	ts.it_value.tv_sec = interval_ms_ / 1000;
	ts.it_value.tv_nsec = (interval_ms_ % 1000000) * 1000000;
	ts.it_interval.tv_sec = interval_ms_ / 1000;
	ts.it_interval.tv_nsec = (interval_ms_ % 1000000) * 1000000;
	
	status = timer_create(CLOCK_MONOTONIC,&se,&timer_id);
	if(status == -1){
		printf("Fail to creat timer\n");
		exit(1);
	}
	status = timer_settime(timer_id,0,&ts,0);
	if(status == -1){
		printf("Fail to set timer\n");
		exit(1);
	}

    std::thread thread(&ROSTypeDataPublisher::runThread, this);
    std::cout << "Publisher running." << std::endl;

    thread.join();

	timer_delete(timer_id);
    
}

bool ROSTypeDataPublisher::publish(bool waitForListener)
{
    if (listener_.first_connected_ || !waitForListener || listener_.matched_ > 0)
    {
        publish_msg_->index(publish_msg_->index() + 1);
        size_t data_size = publish_msg_->pose().size();
        std::string s = "BigData" + std::to_string(publish_msg_->index() % 10);
        publish_msg_->message() = s;
        // strcpy(&publish_msg_->pose()[data_size - s.length() - 1], s.c_str());

        writer_->write(publish_msg_.get());

        return true;
    }
    return false;
}
