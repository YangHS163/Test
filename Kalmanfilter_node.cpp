#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseWithCovariance.h"
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <vector>
using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

vector<Vector3f> state_filtered;
vector<Matrix3f> covariance_filtered;
ros::Publisher chatter_pub;
Vector3f state_first_filtered(0,0,0);
Matrix3f covariance_first_filtered = Matrix3f::Identity();

bool first = true;
Matrix3f A; 
Matrix3f B;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const Pose& msg)
{
   if(first){
	state_filtered.push_back(state_first_filtered);
	covariance_filtered.push_back(covariance_first_filtered);
	first = false;
   }
   //geometry_msgs/PoseWithCovariance temp = msg.pose;
   Matrix3f P;
   Matrix3f Q; //sensed noise
   //There is noise in every dimension, and the white noise is not correlated to each other
   P << 1e-6, 0, 0,
       0, 1e-6, 0,
       0, 0, 1e-6;
   Q << 2, 0, 0,
       0, 2, 0,
       0, 0, 2;
   //ROS_INFO("x: [%f]", msg.position.x);
   //ROS_INFO("y: [%f]", msg.position.y);
   //ROS_INFO("z: [%f]", msg.position.z);
   Vector3f measure;
   measure(0) = msg.position.x;
   measure(1) = msg.position.y;
   measure(2) = msg.position.z;
   Vector3f state_predicted = A * state_filtered.back();
   Matrix3f covariance_predicted = A * covariance_filtered.back() * A.transpose() + P;
   Matrix3f temp = B * covariance_predicted * B.transpose() + Q;
   Matrix3f KalmanGain = covariance_predicted * B * temp.inverse();
   state_filtered.push_back(state_predicted + KalmanGain * (measure - B * state_predicted));
   Matrix3f I = Matrix3f::Identity();
   covariance_filtered.push_back((I - KalmanGain * B)* covariance_predicted);
   Vector3f res = state_filtered.back();
   ROS_INFO("filtered_x: [%f]", res(0));
   ROS_INFO("filtered_y: [%f]", res(1));
   ROS_INFO("filtered_z: [%f]", res(2));
   //ROS_INFO("size: [%d]", state_filtered.size());

  
  geometry_msgs::Pose filtered;
    if(!state_filtered.empty()){
	Vector3f temp = state_filtered.back();
	filtered.position.x = temp(0);
	filtered.position.y = temp(1);
	filtered.position.z = temp(2);
	//state_filtered.pop_back();
	}
   chatter_pub.publish(filtered);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "Kalman_filter_node");
  
  
  A << 1, 0, 0,
     0, 1, 0,
     0, 0, 1;
  B << 1, 0, 0,
     0, 1, 0,
     0, 0, 1;;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  
   ros::NodeHandle n;
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("input_measurement", 10000, chatterCallback);
  chatter_pub = n.advertise<geometry_msgs::Pose>("filtered_results", 10000);
  
  /*while (ros::ok())
  {
    geometry_msgs::Pose filtered;
    if(!state_filtered.empty()){
	Vector3f temp = state_filtered.back();
	filtered.position.x = temp(0);
	filtered.position.y = temp(1);
	filtered.position.z = temp(2);
	//state_filtered.pop_back();
	}
    chatter_pub.publish(filtered);
    loop_rate.sleep();
  }*/

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
