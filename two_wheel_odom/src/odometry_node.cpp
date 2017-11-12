#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <two_wheel_odom/odometry.h>
#include <sessel_otter/MotorTicks.h>

double leftMeters = 0;
double rightMeters = 0;

double ticksPerMeter = 0;
double wheelbase = 0;

void odomUpdate(const sessel_otter::MotorTicks& msg)
{
    leftMeters = double(msg.leftTicks) / ticksPerMeter;
    rightMeters = double(msg.rightTicks) / ticksPerMeter;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::Subscriber sub = n.subscribe("motor_ticks", 1, odomUpdate);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  pn.getParam("ticks_per_meter", ticksPerMeter);
  pn.getParam("wheelbase", wheelbase);

  bool publish_tf = true;
  if (pn.hasParam("publish_tf")) {
      pn.getParam("publish_tf", publish_tf);
  }
  if (publish_tf) {
    ROS_INFO("Publishing odom tf");
  } else {
    ROS_INFO("NOT Publishing odom tf");
  }

  Odometry odometry(wheelbase);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now() - ros::Duration(0.1); //Prevent division by zero

  double lastLeftMeters = 0;
  double lastRightMeters = 0;

  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double deltaLeftMeters = leftMeters - lastLeftMeters;
    double deltaRightMeters = rightMeters - lastRightMeters;

    odometry.update(deltaLeftMeters, deltaRightMeters, dt);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odometry.getTh());

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = odometry.getX();
    odom_trans.transform.translation.y = odometry.getY();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    if (publish_tf) {
        odom_broadcaster.sendTransform(odom_trans);
    }

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = odometry.getX();
    odom.pose.pose.position.y = odometry.getY();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odometry.getVX();
    odom.twist.twist.linear.y = odometry.getVY();
    odom.twist.twist.angular.z = odometry.getVTh();

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    lastLeftMeters = leftMeters;
    lastRightMeters = rightMeters;
    r.sleep();
  }
}
