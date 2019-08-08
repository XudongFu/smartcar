#ifndef dog_robotHW
#define dog_robotHW

#include <hardware_interface/robot_hw.h>
using namespace hardware_interface;

namespace free
{


class dogRobotHW:RobotHW
{

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);

    void read(const ros::Time& time, const ros::Duration& period);

    void write(const ros::Time& time, const ros::Duration& period);
};

}


#endif