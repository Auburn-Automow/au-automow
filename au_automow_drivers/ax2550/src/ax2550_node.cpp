#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <string>

#include "ax2550.h"

AX2550 *ax2550;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ax2550->move(msg->linear.x, msg->angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ax2550_node");
    
    ros::NodeHandle n;
    
    std::string port("/dev/motor_controller");
    
    ax2550 = new AX2550(port);
    
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);
    
    ros::spin();
    
    return 0;
}