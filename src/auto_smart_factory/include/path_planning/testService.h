#include "ros/ros.h"
//#include "auto_smart_factory/testService.h"
#include "auto_smart_factory/RequestNewPath.h"

class Test {
    public:
        Test();
        void function();
        bool handler(auto_smart_factory::RequestNewPath::Request  &req, auto_smart_factory::RequestNewPath::Response &res);

    private:
        ros::ServiceServer server;


};