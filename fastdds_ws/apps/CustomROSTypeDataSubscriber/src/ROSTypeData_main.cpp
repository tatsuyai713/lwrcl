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
 * @file ROSTypeData_main.cpp
 *
 */

#include<stdio.h>
#include<string.h>
#include<unistd.h>
#include "ROSTypeDataSubscriber.h"

#include <fastrtps/Domain.h>
#include <fastrtps/log/Log.h>

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

#define BUFSIZE 255
int main(
        int argc,
        char **argv)
{
    std::cout << "Starting " << std::endl;

    char buf[BUFSIZE];
    int v = readlink("/proc/self/exe", buf, sizeof(buf)); 
    std::string fullpath;
    if (v != -1) {  
        fullpath = std::string(buf);
        size_t pos1;
        pos1 = fullpath.rfind("/");
        if(pos1 != std::string::npos){
            fullpath = fullpath.substr(0, pos1+1);
        }
    }
 
    ROSTypeDataSubscriber subscriber;
    printf("%s\n",fullpath.c_str());
    if (subscriber.init(fullpath + std::string("/config/config.yaml")))
    {
        subscriber.run();
    }
    Domain::stopAll();
    Log::Reset();
    return 0;
}
