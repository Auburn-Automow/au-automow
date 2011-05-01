#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

void poseCallback(const sensor_msgs::ImuConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  tf::Quaternion angles;
  tf::quaternionMsgToTF(msg->orientation, angles);
      
  double roll, pitch, yaw;
  btMatrix3x3(angles).getRPY(roll, pitch,yaw);

  transform.setOrigin( tf::Vector3(0.0,0.0,0.0));
  //  transform.setRotation( tf::createQuaternionFromRPY(roll, pitch, 0.0) );
  transform.setRotation(angles);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "/ground", msg->header.frame_id));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "imu_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/imu/data", 10, &poseCallback);

  ros::spin();
  return 0;
};
