
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <string>
#include <transmission_interface/transmission_parser.h>
#include <control_toolbox/pid.h>
#include <controller_manager/controller_manager.h>
using namespace hardware_interface;
using namespace ros;
using namespace std;
#define var auto
namespace freeFxd
{

class dogRobotHW : RobotHW
{

private:
    enum ControlMethod
    {
        EFFORT,
        POSITION,
        POSITION_PID,
        VELOCITY,
        VELOCITY_PID
    };

    std::vector<std::string> joint_names_;
    std::vector<std::string> joint_types_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    std::vector<double> joint_effort_limits_;
    std::vector<ControlMethod> joint_control_methods_;
    std::vector<control_toolbox::Pid> pid_controllers_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> last_joint_position_command_;
    std::vector<double> joint_velocity_command_;
    hardware_interface::EffortJointInterface ej_interface_;
    hardware_interface::PositionJointInterface pj_interface_;
    hardware_interface::VelocityJointInterface vj_interface_;

public:
    /*
    根据tranmision信息加载资源到接口中去，初始化控制器
    
     */
    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        string robotDesc;
        bool success = ros::param::get("robot_description", robotDesc);
        if (!success)
        {
            ROS_INFO("cannot get robot_description");
            return false;
        }
        std::vector<transmission_interface::TransmissionInfo> transmissions;
        transmission_interface::TransmissionParser::parse(robotDesc, transmissions);
        if (transmissions.size() == 0)
        {
            ROS_WARN("robot_description don't contain any TransmissionInfo");
            return false;
        }
        auto size = transmissions.size();
        joint_names_.resize(size);
        joint_types_.resize(size);

        joint_position_.resize(size);
        joint_velocity_.resize(size);
        joint_effort_.resize(size);

        joint_effort_command_.resize(size);
        joint_position_command_.resize(size);
        joint_velocity_command_.resize(size);
        last_joint_position_command_.resize(size);
        joint_control_methods_.resize(size);
        int i = 0;
        for (var curr = transmissions.begin(); curr != transmissions.end(); curr++)
        {
            string joint;
            if (curr->joints_.size() != 0)
            {
                joint = curr->joints_.front().name_;
            }
            else
            {
                ROS_INFO("there is no joint");
                return false;
            }
            string type = curr->joints_.front().hardware_interfaces_.front();
            JointStateHandle handle(curr->name_, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            /**
             * todo ,not care actutor
             */

            if (type.find("EffortJointInterface") != string::npos)
            {
                joint_control_methods_[i] = EFFORT;
                JointHandle hand(handle, &joint_effort_command_[i]);
                ej_interface_.registerHandle(hand);
            }

            if (type.find("PositionJointInterface") != string::npos)
            {
                joint_control_methods_[i] = POSITION;
                JointHandle hand(handle, &joint_position_command_[i]);
                pj_interface_.registerHandle(hand);
            }

            if (type.find("VelocityJointInterface") != string::npos)
            {
                joint_control_methods_[i] = VELOCITY;
                JointHandle hand(handle, &joint_velocity_command_[i]);
                vj_interface_.registerHandle(hand);
            }
            i++;
        }
        registerInterface(&ej_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&vj_interface_);
    }

    /**
 * 控制器负责调用这些函数
 */
    void read(const ros::Time &time, const ros::Duration &period)
    {
    }

    void write(const ros::Time &time, const ros::Duration &period)
    {
    }
};

} // namespace freeFxd

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot");
    NodeHandle handle;
    freeFxd::dogRobotHW hw;
    hw.init(handle, handle);
    hw.getInterfaceResources("");
    //manager.hw.init(nullptr, nullptr);
    // controller_manager::ControllerManager manager(hw, handle);
    // ros::Rate rate(10);
    // ros::Time lasttime = ros::Time::now();
    // while (true)
    // {
    //     Time now = Time::now();
    //     hw.read(now, now - lasttime);
    //     hw.write(now, now - lasttime);
    //     manager.update(now, now - lasttime);
    //     lasttime=now;
    //     rate.sleep();
    // }
    return 0;
}
