/*
 * Title:	Keyboard Control
 * Description:	Control the car movement using keyboard.
 * xinkai.zhang@huskers.unl.edu
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>   // for publishing on '/turtle1/cmd_vel' topic
#include <unistd.h>
#include <std_msgs/UInt8.h>       
#define ROS_NODE_NAME "keyboard_ctrl"


ros::Publisher keyboard_pub;    // declare the publisher
geometry_msgs::Twist speeds;    // leverage the geometry_msgs::Twist message type. 
uint8_t key_flag = 0;           
uint8_t key_counter = 0;

// callback function that will get called when a new message has arrived on the /turtle1/cmd topic
void cmdCallback( const geometry_msgs::Twist::ConstPtr &msg ) 
{ 
  //set left and right motor speeds to 20 or -20 when up or down arrow is hit. Motor range -100~100
  if (msg -> linear.x != 0) {
     speeds.linear.x = 10 * (int)msg -> linear.x;
  }

  //set left and right motor speeds difference to 26 or -26 when left or right arrow is hit
  if (msg -> angular.z != 0) {
     speeds.angular.z = 13 * (int)msg -> angular.z;
  }

  key_flag = 1;  //set flag to 1 when the keyboard hit is detected
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, ROS_NODE_NAME ); // initialize ROS node
  ros::NodeHandle nh_;

  // Initialize the ROS publisher
  keyboard_pub = nh_.advertise <geometry_msgs::Twist> ( "motorSpeeds", 1, true );

  // Initialize the ROS subscriber
  ros::Subscriber turtle_sub;
  turtle_sub = nh_.subscribe( "/turtle1/cmd_vel", 1, cmdCallback );

  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    if(key_flag == 1)
    {
      key_counter = 0;
    }

    if(key_flag == 0)
    {
      key_counter++;  
    }

    if(key_counter >= 50) // When there's one keyboard hit detected, keep the motor command for 0.5s
    {
      speeds.linear.x = 0.0;
      speeds.angular.z = 0.0;
      key_counter = 0;
    }

    keyboard_pub.publish(speeds);
    key_flag = 0;
    ros::spinOnce();
	  loop_rate.sleep();
  } 
  return 0;
}