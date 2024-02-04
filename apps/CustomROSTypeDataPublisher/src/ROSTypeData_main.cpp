#include "ROSTypeDataPublisher.hpp"

using namespace rcl_like_wrapper;
#define BUFSIZE 255

int main(int argc, char **argv)
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
 
    ROSTypeDataPublisher ros_like_node;
    printf("%s\n",fullpath.c_str());
    if (ros_like_node.init(fullpath + std::string("/config/config.yaml")))
    {
        ros_like_node.run();
    }
    return 0;
}
