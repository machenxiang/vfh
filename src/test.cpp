

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <GeographicLib/Geocentric.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros/frame_tf.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/PositionTarget.h>



int main(int argc, char **argv)
{



	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;


	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);

	ros::Rate rate(20.0);
    mavros_msgs::PositionTarget pos_target;


    while(ros::ok())
    {
    for(int i=0;i<100;i++)
        {
            pos_target.coordinate_frame=1;
            pos_target.type_mask = 1 + 2 + 4+/* 8 + 16 + 32 */ + 64 + 128 + 256 + 512 + 1024 + 2048;
            pos_target.velocity.x = 1;
            pos_target.velocity.y = 0;

            local_pos_pub.publish(pos_target);
            ROS_INFO("vx= %f vy= %f \n",pos_target.velocity.x,pos_target.velocity.y);
            rate.sleep();
        }
            ros::Time last_request=ros::Time::now();
            while (ros::Time::now()-last_request<ros::Duration(5))
            {
            pos_target.coordinate_frame=1;
            pos_target.type_mask = 1 + 2 + 4+/* 8 + 16 + 32 */ + 64 + 128 + 256 + 512 + 1024 + 2048;
            pos_target.velocity.x = 0;
            pos_target.velocity.y = 0;

            local_pos_pub.publish(pos_target);
            ROS_INFO("vx= %f vy= %f \n",pos_target.velocity.x,pos_target.velocity.y);
            rate.sleep();
            }
            


    for(int i=0;i<100;i++)
        {
            pos_target.coordinate_frame=1;
            pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
            pos_target.velocity.x = 0;
            pos_target.velocity.y = 1;
            local_pos_pub.publish(pos_target);
            ROS_INFO("vx= %f vy= %f \n",pos_target.velocity.x,pos_target.velocity.y);
            rate.sleep();
        }


    }



	

	return 0;

}