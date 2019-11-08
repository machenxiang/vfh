#include<ros/ros.h>
//此时头文件依然#include .h文件
#include"vfh.h"

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


uint32_t init_mask = 0;
SectorMap *smap;
geometry_msgs::PoseStamped local_pos;
Eigen::Vector3d current_local_pos;
bool local_pos_updated = false;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	init_mask |= 1 ;
	current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
	local_pos = *msg;
	Point2D pt2d;
	pt2d.x = current_local_pos.x();
	pt2d.y = current_local_pos.y();
	smap->SetUavPosition(pt2d);
	local_pos_updated = true;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	init_mask |= 1 << 1;
	current_state = *msg;
}


mavros_msgs::WaypointList waypoints;
void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg) {
	init_mask |= 1 << 2;
	waypoints = *msg;
}

bool scan_updated = false;
void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//激光雷达数据回调函数
	init_mask |= 1 << 3;
	smap->ComputeCV(msg->ranges);
	scan_updated = true;
}


Eigen::Vector3d current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	init_mask |= 1<<4;
	current_gps = { msg->latitude, msg->longitude, msg->altitude };
}
//更新完成标志位0x1f


int main(int argc,char**argv)
{
    smap=new SectorMap;
    ros::init(argc,argv,"offb_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, local_pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
	ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 100, waypoints_cb);
	ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("laser/scan", 100, scan_cb);
	ros::Subscriber gps_sub = nh.subscribe("mavros/global_position/global", 100, gps_cb);
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);


	while (ros::ok()&&(!current_state.connected))
    {
        ros::spinOnce();
		rate.sleep();
		ROS_INFO("can't connected to mavros");
    }
	ROS_INFO("connected to mavros");

	while(ros::ok())
	{

		if(init_mask==0x1f)
		{
			break;
		}
		ros::spinOnce();
		rate.sleep();
		ROS_INFO("can't update all data");
	}
	ROS_INFO("all data update complete");
//******************************解算坐标到ENU坐标系下**************************************************
	std::vector<geometry_msgs::PoseStamped> pose;
	printf("wp size=%d\n", waypoints.waypoints.size());
	for (int index = 0; index < waypoints.waypoints.size(); index++)
	{
		geometry_msgs::PoseStamped p;

		GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
		//mavros_msgs::WaypointList waypoints;
		Eigen::Vector3d goal_gps(waypoints.waypoints[index].x_lat, waypoints.waypoints[index].y_long, 0);

		Eigen::Vector3d current_ecef;
		
        //将大地坐标转换为地心坐标
		earth.Forward(current_gps.x(), current_gps.y(), current_gps.z(),

		current_ecef.x(), current_ecef.y(), current_ecef.z());

		Eigen::Vector3d goal_ecef;

		earth.Forward(goal_gps.x(), goal_gps.y(), goal_gps.z(),

		goal_ecef.x(), goal_ecef.y(), goal_ecef.z());

		Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
		
		    //将ECEF坐标系转换到ENU坐标系
		Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

		Eigen::Affine3d sp;

		Eigen::Quaterniond q;

		q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())

			* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())

			* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

		sp.translation() = current_local_pos + enu_offset;

		sp.linear() = q.toRotationMatrix();

		Eigen::Vector3d testv(sp.translation());
		p.pose.position.x = testv[0];
		p.pose.position.y = testv[1];
		printf("%f %f\n", testv[0], testv[1]);

		pose.push_back(p);
	}
//*********************************开始跑航点*************************************************
	for (int i = 0; i < pose.size(); i++)
		{
			while (ros::ok()) 
			{
				local_pos_updated = false;
				scan_updated = false;
                //检查更新
				while (ros::ok())
				{
					ros::spinOnce();
					if (local_pos_updated && scan_updated)
						break;
					rate.sleep();
					ROS_INFO("bug1");
				}

				if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
					fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
				{
					ROS_INFO("arrive waypoint %d",i);
					break;
				}
				
				Point2D goal;
				goal.x = pose[i].pose.position.x;
				goal.y = pose[i].pose.position.y;
				//a pointer object
				//速度要使用弧度制，不然会震荡
				float degree=smap->CalculDirection(goal);
				
				float ang=int(ang*(180/PI));
                ang+=360;
                ang=int(ang)%360;
                ROS_INFO("ang in ENU %f without consider front safety\n",ang);

				if(smap->IsFrontSafety(degree))
				{
					//目标方向30°安全
					ROS_INFO("front safty");
				}
				else
				{
					ROS_INFO("front dangerous");
					degree=smap->SelectNewDir(degree);
				}
				

				mavros_msgs::PositionTarget pos_target;
				pos_target.coordinate_frame=1;
				pos_target.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
				pos_target.velocity.x = 0.5*cos(degree);
				pos_target.velocity.y = 0.5*sin(degree);
				pos_target.position.z=1.5;
				local_pos_pub.publish(pos_target);
				ROS_INFO("go to waypoint %d vx= %f vy= %f \n",i,pos_target.velocity.x,pos_target.velocity.y);
				ros::Time last_request = ros::Time::now();
				while (ros::ok()) 
				{
					local_pos_updated = false;
					scan_updated = false;
					while (ros::ok())
					{
						ros::spinOnce();
						if (local_pos_updated && scan_updated)
							break;
							rate.sleep();
						ROS_INFO("bug2");
					}

					if (current_state.mode != "OFFBOARD")
						break;

					if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
						fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
					{
						break;
					}
					if(!(smap->IsFrontSafety(degree)))
					{
						break;
					}

					if (ros::Time::now() - last_request > ros::Duration(4.0))
						break;
					local_pos_pub.publish(pos_target);

			    }
            //这里发布00防止转机头指北
			    pos_target.coordinate_frame=1;
			    pos_target.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
			    pos_target.velocity.x = 0;
			    pos_target.velocity.y = 0;
				pos_target.position.z=1.5;
			    local_pos_pub.publish(pos_target);
		    }


	}
    

	return 0;
}