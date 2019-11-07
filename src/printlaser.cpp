#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

#include<vector>
using namespace std;

vector<double> ranges;

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges.clear();
    int size=msg->ranges.size();
    for(int i=0;i<size;i++)
    {
        ranges.push_back(msg->ranges[i]);

    }
        for(int i=0;i<size;i++)
    {
        
        printf("i=%d %f\n",ranges[i]);
    }
}
int main(int argc, char **argv)
{


	ros::init(argc, argv, "printLaser_node");
	ros::NodeHandle nh;
    ros::Rate rate(20);
	ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("laser/scan", 100, scan_cb);
	while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("*******************************************************************\n");
    }


	return 0;
}
