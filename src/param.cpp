#include <ros/ros.h>
#include <iostream>
using namespace std;
using namespace ros;

int main(int argc, char *argv[])
{
    int x;
    cin>>x;
    ros::init(argc, argv, "param");
    ROS_INFO("args is here");
    for(int i=0;i<argc;i++)
    {
        cout<<*argv[i]<<endl;
        ROS_INFO("args is %s",argv[i]);
    }
    
    cin>>x;
    return 0;
}