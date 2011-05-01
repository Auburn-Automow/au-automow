#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include <string>
#include <cmath>

#include "ax2550.h"

AX2550 *ax2550;
ros::Publisher odom_pub;

static double ENCODER_RESOLUTION = 250*4;
double wheel_circumference = 0.0;
double wheel_base_length = 0.0;
double wheel_diameter = 0.0;
std::string odom_frame_id;

static double A_MAX = 20.0;
static double B_MAX = 20.0;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
ros::Time prev_time;

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if(ax2550 == NULL || !ax2550->isConnected())
        return;
    // Convert mps to rpm
    double A = msg->linear.x;
    double B = msg->angular.z * (wheel_base_length/2.0);
    
    double A_rpm = A * (60.0 / (M_PI*wheel_diameter));
    double B_rpm = B * (60.0 / (M_PI*wheel_diameter));
    
    // Convert rpm to relative
    double A_rel = (A_rpm * 250 * 64) / 58593.75;
    double B_rel = (B_rpm * 250 * 64) / 58593.75;
    
    // ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, B_rel);
    
    // Bounds check
    if(A_rel > A_MAX)
        A_rel = A_MAX;
    if(A_rel < -1*A_MAX)
        A_rel = -1*A_MAX;
    if(B_rel > B_MAX)
        B_rel = B_MAX;
    if(B_rel < -1*B_MAX)
        B_rel = -1*B_MAX;
    
    // ROS_INFO("%f %f", A_rel/127.0, B_rel/127.0);
    
    ax2550->move(A_rel/127.0, B_rel/127.0);
}

void errorMsgCallback(const std::string &msg) {
    ROS_ERROR("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

void encoderCallback(const ros::TimerEvent& e) {
    // Make sure we are connected
    if(!ros::ok() || ax2550 == NULL || !ax2550->isConnected())
        return;
    
    AX2550_ENCODER ax2550_encoder(0,0);
    // Retreive the data
    try {
        ax2550_encoder = ax2550->readEncoders();
        // ROS_INFO("Encoder Data: %d, %d", ax2550_encoder.encoder1, ax2550_encoder.encoder2);
    } catch(std::exception &e) {
        ROS_ERROR("Error reading the Encoders: %s", e.what());
    }
    // Grab the time
    ros::Time now = ros::Time::now();
    
    // Convert to mps for each wheel from delta encoder ticks
    double left_v = ax2550_encoder.encoder1 * wheel_circumference / ENCODER_RESOLUTION;
    double right_v = ax2550_encoder.encoder2 * wheel_circumference / ENCODER_RESOLUTION;
    
    double v = 0.0;
    double w = 0.0;
    
    // Do inverse kinematics
    if(left_v == right_v) {
        v = left_v;
        w = 0.0;
    } else if(left_v == -1*right_v) {
        v = 0.0;
        w = (2.0/wheel_base_length) * left_v;
    } else {
        v = (left_v+right_v)/2.0;
        w = 2.0*(right_v-left_v)/wheel_base_length;
    }
    
    // Accumulate
    prev_w += w;
    prev_x += v * cos(prev_w);
    prev_y += v * sin(prev_w);
    double delta_time = (now - prev_time).toSec();
    prev_time = now;
    
    ROS_INFO("%f", prev_w);
    
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(prev_w);
    
    // Populate the msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.pose.pose.position.x = prev_x;
    odom_msg.pose.pose.position.y = prev_y;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.pose.covariance[0] = 1e-5;
    odom_msg.pose.covariance[7] = 1e-5;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = 1e-3;
    
    odom_msg.twist.twist.linear.x = v/delta_time;
    odom_msg.twist.twist.angular.z = w/delta_time;
    
    odom_pub.publish(odom_msg);
    
    // TODO: Add TF broadcaster
}

int main(int argc, char **argv) {
    // Node setup
    ros::init(argc, argv, "ax2550_node");
    ros::NodeHandle n;
    prev_time = ros::Time::now();
    
    // Serial port parameter
    std::string port;
    n.param("serial_port", port, std::string("/dev/motor_controller"));
    
    // Wheel diameter parameter
    n.param("wheel_diameter", wheel_diameter, 0.3048);
    
    wheel_circumference = wheel_diameter * M_PI;
    
    // Wheel base length
    n.param("wheel_base_length", wheel_base_length, 0.9144);
    
    // Odom Frame id parameter
    n.param("odom_frame_id", odom_frame_id, std::string("odom"));
    
    // Setup Encoder polling
    double encoder_poll_rate;
    n.param("encoder_poll_rate", encoder_poll_rate, 50.0);
    ros::Timer encoder_timer = n.createTimer(ros::Duration(1.0/encoder_poll_rate), encoderCallback);
    
    // Odometry Publisher
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
    
    // cmd_vel Subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);
    
    while(ros::ok()) {
        ROS_INFO("AX2550 connecting to port %s", port.c_str());
        try {
            ax2550 = new AX2550(port);
            ax2550->setErrorMsgCallback(errorMsgCallback);
            ax2550->setInfoMsgCallback(infoMsgCallback);
            ax2550->connect();
        } catch(std::exception &e) {
            ROS_ERROR("Failed to connect to the AX2550: %s", e.what());
        }
        while(ax2550->isConnected()) {
            if(!ros::ok())
                break;
            ros::spinOnce();
        }
        delete ax2550;
        ax2550 = NULL;
        if(!ros::ok())
            break;
        ROS_WARN("Will try to resync in 1 sec.");
        ros::Duration(1).sleep();
    }
    
    return 0;
}
