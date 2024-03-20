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
 * @file ImageSubscriber.cpp
 *
 */

#include "ImageSubscriber.h"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <opencv2/opencv.hpp>

using namespace eprosima::fastdds::dds;
using namespace std;
using namespace cv;

const std::string ABSTRACT_ENCODING_PREFIXES[] = {"8UC", "8SC", "16UC", "16SC", "32SC", "32FC", "64FC"};
const std::string BAYER_BGGR16 = "bayer_bggr16";
const std::string BAYER_BGGR8 = "bayer_bggr8";
const std::string BAYER_GBRG16 = "bayer_gbrg16";
const std::string BAYER_GBRG8 = "bayer_gbrg8";
const std::string BAYER_GRBG16 = "bayer_grbg16";
const std::string BAYER_GRBG8 = "bayer_grbg8";
const std::string BAYER_RGGB16 = "bayer_rggb16";
const std::string BAYER_RGGB8 = "bayer_rggb8";
const std::string BGR16 = "bgr16";
const std::string BGR8 = "bgr8";
const std::string BGRA16 = "bgra16";
const std::string BGRA8 = "bgra8";
const std::string MONO16 = "mono16";
const std::string MONO8 = "mono8";
const std::string RGB16 = "rgb16";
const std::string RGB8 = "rgb8";
const std::string RGBA16 = "rgba16";
const std::string RGBA8 = "rgba8";
const std::string TYPE_16SC1 = "16SC1";
const std::string TYPE_16SC2 = "16SC2";
const std::string TYPE_16SC3 = "16SC3";
const std::string TYPE_16SC4 = "16SC4";
const std::string TYPE_16UC1 = "16UC1";
const std::string TYPE_16UC2 = "16UC2";
const std::string TYPE_16UC3 = "16UC3";
const std::string TYPE_16UC4 = "16UC4";
const std::string TYPE_32FC1 = "32FC1";
const std::string TYPE_32FC2 = "32FC2";
const std::string TYPE_32FC3 = "32FC3";
const std::string TYPE_32FC4 = "32FC4";
const std::string TYPE_32SC1 = "32SC1";
const std::string TYPE_32SC2 = "32SC2";
const std::string TYPE_32SC3 = "32SC3";
const std::string TYPE_32SC4 = "32SC4";
const std::string TYPE_64FC1 = "64FC1";
const std::string TYPE_64FC2 = "64FC2";
const std::string TYPE_64FC3 = "64FC3";
const std::string TYPE_64FC4 = "64FC4";
const std::string TYPE_8SC1 = "8SC1";
const std::string TYPE_8SC2 = "8SC2";
const std::string TYPE_8SC3 = "8SC3";
const std::string TYPE_8SC4 = "8SC4";
const std::string TYPE_8UC1 = "8UC1";
const std::string TYPE_8UC2 = "8UC2";
const std::string TYPE_8UC3 = "8UC3";
const std::string TYPE_8UC4 = "8UC4";
const std::string YUV422 = "yuv422";

static inline int numChannels(const std::string &encoding)
{
    // First do the common-case encodings
    if (encoding == MONO8 ||
        encoding == MONO16)
        return 1;
    if (encoding == BGR8 ||
        encoding == RGB8 ||
        encoding == BGR16 ||
        encoding == RGB16)
        return 3;
    if (encoding == BGRA8 ||
        encoding == RGBA8 ||
        encoding == BGRA16 ||
        encoding == RGBA16)
        return 4;
    if (encoding == BAYER_RGGB8 ||
        encoding == BAYER_BGGR8 ||
        encoding == BAYER_GBRG8 ||
        encoding == BAYER_GRBG8 ||
        encoding == BAYER_RGGB16 ||
        encoding == BAYER_BGGR16 ||
        encoding == BAYER_GBRG16 ||
        encoding == BAYER_GRBG16)
        return 1;

    // Now all the generic content encodings
    // TODO: Rewrite with regex when ROS supports C++11
    for (size_t i = 0; i < sizeof(ABSTRACT_ENCODING_PREFIXES) / sizeof(*ABSTRACT_ENCODING_PREFIXES); i++)
    {
        std::string prefix = ABSTRACT_ENCODING_PREFIXES[i];
        if (encoding.substr(0, prefix.size()) != prefix)
            continue;
        if (encoding.size() == prefix.size())
            return 1; // ex. 8UC -> 1
        int n_channel = atoi(encoding.substr(prefix.size(),
                                             encoding.size() - prefix.size())
                                 .c_str()); // ex. 8UC5 -> 5
        if (n_channel != 0)
            return n_channel; // valid encoding string
    }

    if (encoding == YUV422)
        return 2;

    return -1;
}

static inline int bitDepth(const std::string &encoding)
{
    if (encoding == MONO16)
        return 16;
    if (encoding == MONO8 ||
        encoding == BGR8 ||
        encoding == RGB8 ||
        encoding == BGRA8 ||
        encoding == RGBA8 ||
        encoding == BAYER_RGGB8 ||
        encoding == BAYER_BGGR8 ||
        encoding == BAYER_GBRG8 ||
        encoding == BAYER_GRBG8)
        return 8;

    if (encoding == MONO16 ||
        encoding == BGR16 ||
        encoding == RGB16 ||
        encoding == BGRA16 ||
        encoding == RGBA16 ||
        encoding == BAYER_RGGB16 ||
        encoding == BAYER_BGGR16 ||
        encoding == BAYER_GBRG16 ||
        encoding == BAYER_GRBG16)
        return 16;

    // Now all the generic content encodings
    // TODO: Rewrite with regex when ROS supports C++11
    for (size_t i = 0; i < sizeof(ABSTRACT_ENCODING_PREFIXES) / sizeof(*ABSTRACT_ENCODING_PREFIXES); i++)
    {
        std::string prefix = ABSTRACT_ENCODING_PREFIXES[i];
        if (encoding.substr(0, prefix.size()) != prefix)
            continue;
        if (encoding.size() == prefix.size())
            return atoi(prefix.c_str()); // ex. 8UC -> 8
        int n_channel = atoi(encoding.substr(prefix.size(),
                                             encoding.size() - prefix.size())
                                 .c_str()); // ex. 8UC10 -> 10
        if (n_channel != 0)
            return atoi(prefix.c_str()); // valid encoding string
    }

    if (encoding == YUV422)
        return 8;

    return -1;
}

int getCvType(const std::string &encoding)
{
    // Check for the most common encodings first
    if (encoding == BGR8)
        return CV_8UC3;
    if (encoding == MONO8)
        return CV_8UC1;
    if (encoding == RGB8)
        return CV_8UC3;
    if (encoding == MONO16)
        return CV_16UC1;
    if (encoding == BGR16)
        return CV_16UC3;
    if (encoding == RGB16)
        return CV_16UC3;
    if (encoding == BGRA8)
        return CV_8UC4;
    if (encoding == RGBA8)
        return CV_8UC4;
    if (encoding == BGRA16)
        return CV_16UC4;
    if (encoding == RGBA16)
        return CV_16UC4;

    // For bayer, return one-channel
    if (encoding == BAYER_RGGB8)
        return CV_8UC1;
    if (encoding == BAYER_BGGR8)
        return CV_8UC1;
    if (encoding == BAYER_GBRG8)
        return CV_8UC1;
    if (encoding == BAYER_GRBG8)
        return CV_8UC1;
    if (encoding == BAYER_RGGB16)
        return CV_16UC1;
    if (encoding == BAYER_BGGR16)
        return CV_16UC1;
    if (encoding == BAYER_GBRG16)
        return CV_16UC1;
    if (encoding == BAYER_GRBG16)
        return CV_16UC1;

        // Check all the generic content encodings
#define CHECK_ENCODING(code)     \
    if (encoding == TYPE_##code) \
    return CV_##code /***/
#define CHECK_CHANNEL_TYPE(t) \
    CHECK_ENCODING(t##1);     \
    CHECK_ENCODING(t##2);     \
    CHECK_ENCODING(t##3);     \
    CHECK_ENCODING(t##4);     \
    /***/

    CHECK_CHANNEL_TYPE(8UC);
    CHECK_CHANNEL_TYPE(8SC);
    CHECK_CHANNEL_TYPE(16UC);
    CHECK_CHANNEL_TYPE(16SC);
    CHECK_CHANNEL_TYPE(32SC);
    CHECK_CHANNEL_TYPE(32FC);
    CHECK_CHANNEL_TYPE(64FC);

#undef CHECK_CHANNEL_TYPE
#undef CHECK_ENCODING

    exit(-1);
}

// Converts a ROS Image to a cv::Mat by sharing the data or changing its endianness if needed
cv::Mat matFromImage(const sensor_msgs::msg::Image &source)
{
    int source_type = getCvType(source.encoding());
    int byte_depth = bitDepth(source.encoding()) / 8;
    int num_channels = numChannels(source.encoding());

    if (source.step() < source.width() * byte_depth * num_channels)
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: step < width * byte_depth * num_channels  or  " << source.step() << " != " << source.width() << " * " << byte_depth << " * " << num_channels;
        exit(-1);
    }

    if (source.height() * source.step() != source.data().size())
    {
        std::stringstream ss;
        ss << "Image is wrongly formed: height * step != size  or  " << source.height() << " * " << source.step() << " != " << source.data().size();
        exit(-1);
    }

    // If the endianness is the same as locally, share the data
    cv::Mat mat(source.height(), source.width(), source_type, const_cast<uchar *>(&source.data()[0]), source.step());

    if (byte_depth == 1)
    {
        return mat;
    }

    // Otherwise, reinterpret the data as bytes and switch the channels accordingly
    mat = cv::Mat(source.height(), source.width(), CV_MAKETYPE(CV_8U, num_channels * byte_depth),
                  const_cast<uchar *>(&source.data()[0]), source.step());
    cv::Mat mat_swap(source.height(), source.width(), mat.type());

    std::vector<int> fromTo;
    fromTo.reserve(num_channels * byte_depth);
    for (int i = 0; i < num_channels; ++i)
    {
        for (int j = 0; j < byte_depth; ++j)
        {
            fromTo.push_back(byte_depth * i + j);
            fromTo.push_back(byte_depth * i + byte_depth - 1 - j);
        }
    }
    cv::mixChannels(std::vector<cv::Mat>(1, mat), std::vector<cv::Mat>(1, mat_swap), fromTo);

    // Interpret mat_swap back as the proper type
    mat_swap.reshape(num_channels);

    return mat_swap;
}

ImageSubscriber::ImageSubscriber()
    : participant_(nullptr), subscriber_(nullptr), topic_(nullptr), reader_(nullptr), type_(new sensor_msgs::msg::ImagePubSubType())
{
}

bool ImageSubscriber::init(
    bool use_env)
{
    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("Participant_sub");
    // pqos.wire_protocol().builtin.readerHistoryMemoryPolicy =
    //     eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
    // pqos.wire_protocol().builtin.writerHistoryMemoryPolicy =
    //     eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    auto factory = DomainParticipantFactory::get_instance();

    if (use_env)
    {
        factory->load_profiles();
        factory->get_default_participant_qos(pqos);
    }

    participant_ = factory->create_participant(0, pqos);

    if (participant_ == nullptr)
    {
        return false;
    }

    // REGISTER THE TYPE
    type_.register_type(participant_);

    // CREATE THE SUBSCRIBER
    SubscriberQos sqos = SUBSCRIBER_QOS_DEFAULT;

    if (use_env)
    {
        participant_->get_default_subscriber_qos(sqos);
    }

    subscriber_ = participant_->create_subscriber(sqos, nullptr);

    if (subscriber_ == nullptr)
    {
        return false;
    }

    // CREATE THE TOPIC
    TopicQos tqos = TOPIC_QOS_DEFAULT;

    if (use_env)
    {
        participant_->get_default_topic_qos(tqos);
    }

    topic_ = participant_->create_topic(
        "rt/repub_raw",
        type_->getName(),
        tqos);

    if (topic_ == nullptr)
    {
        return false;
    }

    // CREATE THE READER
    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
    rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;

    rqos.endpoint().history_memory_policy =
        eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    if (use_env)
    {
        subscriber_->get_default_datareader_qos(rqos);
    }

    reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);

    if (reader_ == nullptr)
    {
        return false;
    }

    return true;
}

ImageSubscriber::~ImageSubscriber()
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

void ImageSubscriber::SubListener::on_subscription_matched(
    DataReader *,
    const SubscriptionMatchedStatus &info)
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

void ImageSubscriber::SubListener::on_data_available(
    DataReader *reader)
{
    SampleInfo info;
    if (reader->take_next_sample(&hello_, &info) == ReturnCode_t::RETCODE_OK)
    {
        if (info.instance_state == ALIVE_INSTANCE_STATE)
        {
            samples_++;
            // Print your structure data here.
            std::cout << "Message " << hello_.header().frame_id() << " " << hello_.header().stamp().sec() << " RECEIVED" << std::endl;

            cv::Mat cv_image; // = cv::Mat(hello_.width(), hello_.height(), CV_8U, &hello_.data);
            cv_image = matFromImage(hello_);
            cv::namedWindow("Gray_image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Gray_image", cv_image);

            cv::waitKey(1);
        }
    }
}

void ImageSubscriber::run()
{
    std::cout << "Subscriber running. Please press enter to stop the Subscriber" << std::endl;
    std::cin.ignore();
}

void ImageSubscriber::run(
    uint32_t number)
{
    std::cout << "Subscriber running until " << number << "samples have been received" << std::endl;
    while (number > listener_.samples_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
