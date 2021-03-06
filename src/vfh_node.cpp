#include "sectormap.h"

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

#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/SetMode.h>

int arm_flag,offboard_flag;

SectorMap *smap;
uint32_t init_mask = 0;

Eigen::Vector3d current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	init_mask |= 1;
	current_gps = { msg->latitude, msg->longitude, msg->altitude };
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	init_mask |= 1 << 1;
	current_state = *msg;
}
//<<移位操作
mavros_msgs::HomePosition home_pos;
void home_pos_cb(const mavros_msgs::HomePosition::ConstPtr& msg) {
	init_mask |= 1 << 2;
	home_pos = *msg;
}
//此时的local pose不是经纬度而是
geometry_msgs::PoseStamped local_pos;
Eigen::Vector3d current_local_pos;
bool local_pos_updated = false;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	init_mask |= 1 << 3;
	current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
	local_pos = *msg;
	Point2D pt2d;
	pt2d.x = current_local_pos.x();
	pt2d.y = current_local_pos.y();
	smap->SetUavPosition(pt2d);
	local_pos_updated = true;
}

mavros_msgs::WaypointList waypoints;
void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg) {
	init_mask |= 1 << 4;
	waypoints = *msg;
}

bool scan_updated = false;
void scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//激光雷达数据回调函数
	init_mask |= 1 << 5;
	smap->ComputeMV(msg->ranges);
	scan_updated = true;
}

bool heading_updated = false;
void heading_cb(const std_msgs::Float64::ConstPtr &msg)
{
	//磁罗盘角度单位为度
	init_mask |= 1 << 6;
	smap->SetUavHeading(msg->data);
	heading_updated = true;
}

int main(int argc,char** argv)
{
    smap = new SectorMap;

	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, local_pos_cb);
	ros::Subscriber home_pos_sub = nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 100, home_pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
	ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 100, waypoints_cb);
	ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 100, scan_cb);
	ros::Subscriber gps_sub = nh.subscribe("mavros/global_position/global", 100, gps_cb);
	ros::Subscriber heading_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 100, heading_cb);
	ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);
    ros::ServiceClient arm_client=nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client=nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Rate rate(20.0);
    //****************************************解锁模式设置**********************************
    while (!current_state.connected)
    {
       ROS_INFO("not connected");
       ros::spinOnce();
       rate.sleep();
    }
    ROS_INFO("succesee connected to mavros");


    mavros_msgs::CommandBool arm_cmd;
    if(!current_state.armed)
    {
       ROS_INFO("not arm,please input 1 for arm or using RC arm"); 
       cin>>arm_flag;
       while (!current_state.armed)
       {
            if(arm_flag==1)
            {
                arm_cmd.request.value=true;
                arm_client.call(arm_cmd);
                rate.sleep();
                ros::spinOnce();
            }
       }
    }
    ROS_INFO("armed");
    //手动解锁
    
    
    ROS_INFO("please input 1 for offboard");
    cin>>offboard_flag;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( current_state.mode != "OFFBOARD")
    {

        if( set_mode_client.call(offb_set_mode) &&offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
    }

    // mavros_msgs::PositionTarget vel;
    // vel.coordinate_frame=1;
    // vel.type_mask=1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
    // vel.velocity.x=0;
    // vel.velocity.y=1;
    // vel.velocity.z=0;
    // while(ros::ok())
    // {
    //     local_pos_pub.publish(vel);
    //     //控制指令的发布速度必须快于2hz
    //     rate.sleep();
    //     ROS_INFO("publish vel");
    // }

    //*********************************航点保存***************************************
    	while (ros::ok() && !current_state.connected) {
		ros::spinOnce();
		rate.sleep();
	}
	//check the callback function if has correct carry out
	while (ros::ok())
	{
		//7F ‭0111 1111‬
		if (init_mask == 0x7f)
			break;

		ros::spinOnce();
		rate.sleep();
	}
	printf("init ok!\n");

	while (ros::ok())
	{
		// while (ros::ok() && current_state.mode != "OFFBOARD")
		// {
		// 	ros::spinOnce();
		// 	rate.sleep();
        //     printf("failed offboard\n")
		// }
		printf("offboard ok\n");
		//**********************************************pose vextor容器***********************************************
		std::vector<geometry_msgs::PoseStamped> pose;
		printf("wp size=%d\n", waypoints.waypoints.size());
		for (int index = 0; index < waypoints.waypoints.size(); index++)
		{
			geometry_msgs::PoseStamped p;
			//将大地坐标下的经纬度转换到地心坐标系的m，z轴指向北极，x轴零经度，零纬度。也就是ECEF坐标系

		    //声明了一个类 earth类的实例化
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
	
		    // 输入   in	local ECEF coordinates [m]
			//  输入 map_origin	geodetic origin [lla]
		
		    //将ECEF坐标系转换到ENU坐标系
			Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps);

			Eigen::Affine3d sp;

			Eigen::Quaterniond q;

			q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())

				* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())

				* Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

			sp.translation() = current_local_pos + enu_offset;

			sp.linear() = q.toRotationMatrix();
			//*******************************往vector容器存数据****************************************
			/*
			Represents a translation transformation.
			*/
			Eigen::Vector3d testv(sp.translation());
			p.pose.position.x = testv[0];
			p.pose.position.y = testv[1];
			printf("%f %f\n", testv[0], testv[1]);
			//std::vector<geometry_msgs::PoseStamped> pose p
			//store target points, the point come from the QGC mission
			pose.push_back(p);
		}

		for (int i = 1; i < pose.size(); i++)
		{
			while (ros::ok()) {
				local_pos_updated = false;
				scan_updated = false;
				heading_updated = false;

				while (ros::ok())
				{
					ros::spinOnce();
					if (local_pos_updated && scan_updated && heading_updated)
						break;
					rate.sleep();
				}

				if (current_state.mode != "OFFBOARD")
					break;

				if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
					fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
				{
					break;
				}

				Point2D goal;
				goal.x = pose[i].pose.position.x;
				goal.y = pose[i].pose.position.y;
				//a pointer object
				float direction = smap->CalculDirection(goal);

				if (direction >= -0.5)
				{
					if (direction > 180)
					{
						direction -= 360;
					}
					//the vehicle move in ENU coordinate

					float arc = 3.1415 / 180 * direction;

					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
				
					pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0.5 * cos(arc);
					pos_target.velocity.y = 0.5 * sin(arc);
					local_pos_pub.publish(pos_target);

					ros::Time last_request = ros::Time::now();
					while (ros::ok()) {
						local_pos_updated = false;
						scan_updated = false;
						heading_updated = false;

						while (ros::ok())
						{
							ros::spinOnce();
							if (local_pos_updated && scan_updated && heading_updated)
								break;
							rate.sleep();
						}

						if (current_state.mode != "OFFBOARD")
							break;

						if (fabs(local_pos.pose.position.x - pose[i].pose.position.x) < 1.0 &&
							fabs(local_pos.pose.position.y - pose[i].pose.position.y) < 1.0)
						{
							break;
						}

						if (ros::Time::now() - last_request > ros::Duration(4.0))
							break;

						if (smap->IsFrontSafety() == false)
							break;

						local_pos_pub.publish(pos_target);
					}
				}
				else
				{
					mavros_msgs::PositionTarget pos_target;
					pos_target.coordinate_frame = 1;
					pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
					pos_target.velocity.x = 0;
					pos_target.velocity.y = 0;
					local_pos_pub.publish(pos_target);
				}
			}
		}

		mavros_msgs::PositionTarget pos_target;
		pos_target.coordinate_frame = 1;
		pos_target.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		pos_target.velocity.x = 0;
		pos_target.velocity.y = 0;
		local_pos_pub.publish(pos_target);

		printf("task over\n");
		while (ros::ok() && current_state.mode == "OFFBOARD")
		{
			ros::spinOnce();
			rate.sleep();
		}
	}


   
   
    // mavros_msgs::PositionTarget vel;
    // vel.coordinate_frame=1;
    // vel.type_mask=1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
    // vel.velocity.x=0;
    // vel.velocity.y=1;
    // vel.velocity.z=0;
    // while(ros::ok())
    // {
    //     vel_pub.publish(vel);
    //     //控制指令的发布速度必须快于2hz
    //     rate.sleep();
    //     ROS_INFO("publish vel");
    // }
    return 0;
}