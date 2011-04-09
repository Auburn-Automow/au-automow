#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <string>

#include "ax2550.h"

AX2550 *ax2550;
ros::Publisher twist_pub;

static double RPMS_TO_MPS = 0.3048*M_PI/60.0;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // TODO: map linear m/s and rad/s into -1:1 for both speed and direction
    ax2550->move(msg->linear.x, msg->angular.z);
}

void errorMsgCallback(const std::string &msg) {
    ROS_ERROR("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

void rpmCallback(const ros::TimerEvent& e) {
    try {
        geometry_msgs::Twist msg;
        // msg.header.stamp = ros::Time::now();
        AX2550_RPM ax2550_rpm = ax2550->readRPM();
        msg.linear.x = ax2550_rpm.rpm1*RPMS_TO_MPS;
        msg.linear.y = ax2550_rpm.rpm2*RPMS_TO_MPS;
        twist_pub.publish(msg);
        
        // ROS_INFO("Wheel RPM's, #1: %d, #2: %d", ax2550_rpm.rpm1, ax2550_rpm.rpm2);
    } catch(std::exception &e) {
        ROS_ERROR("Error reading RPMs: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ax2550_node");
    
    ros::NodeHandle n;
    
    std::string port("/dev/tty.usbserial-FTE3Q38X");
    
    ax2550 = new AX2550(port);
    ax2550->setErrorMsgCallback(errorMsgCallback);
    ax2550->setInfoMsgCallback(infoMsgCallback);
    ax2550->connect();
    
    twist_pub = n.advertise<geometry_msgs::Twist>("wheels", 50);
    
    ros::Timer rpm_timer = n.createTimer(ros::Duration(1.0/50), rpmCallback);
    
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);
    
    ros::spin();
    
    return 0;
}