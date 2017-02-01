#include "ros/ros.h"
#include "gradientpkg/mesure.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv,"givedata ");

  ros::NodeHandle n;
  ros::Publisher donnees_pub = n.advertise<gradientpkg::mesure>("donnees",1000);

  ros::Rate loop_rate(10);

  float count=0;
  float px=0,py=0,m=0;

  while(ros::ok())
  {
  
  gradientpkg::mesure msg;
  msg.x=px;
  msg.y=py;
  msg.a=m; 
  
  ROS_INFO("position x:%f y:%f %f",msg.x,msg.y,msg.a);

  donnees_pub.publish(msg);

  ros::spinOnce();

  loop_rate.sleep();

  
  py+=2.0;
  count++;
  m+=2*count;
  }

  return 0;
}
