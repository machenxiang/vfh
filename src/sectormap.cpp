#include "sectormap.h"

#include <cmath>
#include <algorithm>

void SectorMap::SetUavPosition(Point2D& uav) {
	Uavp.x = uav.x;
	Uavp.y = uav.y;
}
//compass heading local ned
//ned to enu
void SectorMap::SetUavHeading(float hd) {
	heading = hd;
	heading += 270;
	heading = (int)heading % 360;
	heading = 360 - heading;
}

void SectorMap::ComputeMV(vector<float> r) {
	float dist[360] = { 0 };
	ranges.clear();
	map_cv.clear();
	//r为激光雷达原始数据长度
	int range_size = r.size();

	for (size_t i = 0; i < range_size; i++)
	{
		//A non-zero value (true) if x is a NaN value; and zero (false) otherwise.

		//isinf A non-zero value (true) if x is an infinity; and zero (false) otherwise.
		//判断数据有效性
		if (!std::isnan(r[i]) && !std::isinf(r[i]))
		{
			float scan_distance = r[i];
			//judge the scan range in which sector
			//floor take an  integer down
			//判断这束激光属于12个30°扇区中的哪一个
			int sector_index = std::floor((i*angle_resolution) / sector_value);
			if (scan_distance >= scan_distance_max || scan_distance < scan_distance_min)
				scan_distance = 0;
			else
				scan_distance = scan_distance_max - scan_distance;
			//divided into 12 mesh,and put the ooo in the mesh
			dist[sector_index] += scan_distance;
		}
		ranges.push_back(r[i]);
	}


	for (int j = 0; j < (int)(360 / sector_value); j++)
	{
		map_cv.push_back(dist[j]);
	}
}

bool SectorMap::IsFrontSafety()
{
	float goal_sector = (int)(0 - (sector_value - sector_scale) + 360) % 360;
	int start_index = goal_sector / angle_resolution;
	float scan_distance = 0;
	for (int i = 0; i < (sector_value - sector_scale) * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	if (scan_distance < 0.1)
	{
		return true;
	}

	return false;
}
//the vehicle in ENU ,ENU is right hand cartesian coordinate
//we firsth change the coordinate into left hand cartesian coordinate
//the laser is in left hand coordinate,the target point is in right hand cartesian coordiante,so we should change the target point
//into left hand cartesian coordiante,to know the real laser index value
float SectorMap::CalculDirection(Point2D& goal) {
	float ori;
	//Compute arc tangent with two parameters
	//return Principal arc tangent of y/x, in the interval [-pi,+pi] radians.
	//One radian is equivalent to 180/PI degrees.
	//Uavp为小车当前位置
	float G_theta = atan2((goal.y - Uavp.y), (goal.x - Uavp.x));
	float goal_ori = G_theta * 180 / PI;
	if (goal_ori < 0)
	{
		goal_ori += 360;
	}
	//heading = 90
	//此时在看激光雷达的坐标系而是使用laserscan的坐标系
	goal_ori -= heading;
	goal_ori += 360;
	goal_ori = (int)goal_ori % 360;

	float goal_sector = (int)(goal_ori - sector_value + 360) % 360;
	int start_index = goal_sector / angle_resolution;
	float scan_distance = 0;
	for (int i = 0; i < sector_value * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	if (scan_distance < 0.1)
	{
		ori = goal_ori;
		ori += heading;
		ori = (int)ori % 360;

		return ori;
	}

	vector<int> mesh;
	for (int i = 0; i < map_cv.size(); i++)
	{
		if (map_cv[i] < 0.1)
			mesh.push_back(0);
		else if (map_cv[i] >= 0.1 && map_cv[i] < 0.3)
			mesh.push_back(2);
		else
			mesh.push_back(4);
	}

	vector<float> cand_dir;
	//if mesh doesn't feed this condition ,it will stop
	//you can see it obs_avoid_node
	//认为两个30°扇区均为0即是小于0.1则为安全扇区
	for (int j = 0; j < mesh.size(); j++)
	{
		if (j == mesh.size() - 1)
		{
			if (mesh[0] + mesh[mesh.size() - 1] == 0)
				cand_dir.push_back(0.0);
		}
		else
		{
			if (mesh[j] + mesh[j + 1] == 0)
				cand_dir.push_back((j + 1)*sector_value);
		}
	}
    //auto指针的for循环
	//选择和目标点偏差小的扇区
	if (cand_dir.size() != 0) {
		vector<float> delta;
		for (auto &dir_ite : cand_dir) {
			float delte_theta1 = fabs(dir_ite - goal_ori);
			float delte_theta2 = 360 - delte_theta1;
			//取小的哪一个
			float delte_theta = delte_theta1 < delte_theta2 ? delte_theta1 : delte_theta2;
			delta.push_back(delte_theta);
		}
		//Return smallest element in range
		//calculate the index of the direcion cloest to the goal
		int min_index = min_element(delta.begin(), delta.end()) - delta.begin();
		printf("min_index=%d \n",min_index);
		ori = cand_dir.at(min_index);

		ori += heading;
		ori = (int)ori % 360;

		return ori;
	}

	return -1;
}
float SectorMap::ComputeAngle(Point2D p)
{
	float dir1=atan2((p.y-Uavp.y),(p.x-Uavp.x));
	float angle=dir1*(1/PI)*180;
	if (angle<0)
	{
		angle=angle+360;
	}
	float move_angle=angle;
	//从ros的ENU坐标系到激光雷达的的坐标系
	//在激光雷达坐标系判断前方是否安全
	angle+=180;
	angle=int(angle)%360;

	float goal_sector=(int)(angle-sector_value+360)%360;
	int start_index = goal_sector / angle_resolution;
	printf("start_index %d\n",start_index);
	float scan_distance = 0;
	for (int i = 0; i < sector_value * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	printf("scan_distance=%f\n",scan_distance);
	//如果安全直行否则选择扇区
		if (scan_distance < 0.1)
	{
		printf("safe\n");


	}
	else
	{
		//航行方向不安全，开始细分扇区
		printf("dangerous,change the way\n");
		move_angle=SwitchVally(map_cv,move_angle);

	}

	
	
	return move_angle;
}

void SectorMap::PrintLaser(vector<double> laser)
{
	for(int i;i<laser.size();i++)
	{
		printf("i=%d %f\n",i,laser[i]);
	}
}

void SectorMap::ComputeCV(vector<float> r)
{
	ranges.clear();
	map_cv.clear();
	float dist[360]={0};
	for(int i=0;i<r.size();i++)
	{

		if (!std::isnan(r[i]) && !std::isinf(r[i]))
			{
				float scan_distance = r[i];
				//judge the scan range in which sector
				//floor take an  integer down
				//判断这束激光属于12个30°扇区中的哪一个
				int sector_index = floor((i*angle_resolution) / sector_value1);
				if (scan_distance >= scan_distance_max || scan_distance < scan_distance_min)
					scan_distance = 0;
				else
					scan_distance = scan_distance_max - scan_distance;
				//divided into 12 mesh,and put the ooo in the mesh
				dist[sector_index] += scan_distance;
			}
		ranges.push_back(r[i]);

	}
	for(int j=0;j<(int)(360/sector_value1);j++)
		{
			map_cv.push_back(dist[j]);
		}

}

bool SectorMap::IsValleySafe(float v)
{
	if(v<0.1)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}
float SectorMap::SwitchVally(vector<float> v,float ang)
{
	//输入ENU坐标系下的目标航向
	vector<int> index;
	vector<int> final_index;
	vector<float> differnce;
	float final_degree=0;
	int final_indexNum;
	for(int i=0;i<(int)(360/sector_value1);i++)
	{
		if(IsValleySafe(map_cv[i]))
		{
			index.push_back(i);
			printf("safe index=%d\n",i);
		}
	}
	for(int j=0;j<index.size();j++)
	{

		int i=(index[j]-1+(int)(360/sector_value1))%(int)(360/sector_value1);
		printf("i=%d\n",i);

		int k=(index[j]+1+(int)(360/sector_value1))%(int)(360/sector_value1);
        printf("k=%d\n",k);
		if(IsValleySafe(map_cv[i])&&IsValleySafe(map_cv[index[j]])&&IsValleySafe(map_cv[k]))
		{
			final_index.push_back(index[j]);
			printf("candicate index=%d\n",index[j]);
		}
	}
	//此时是laser_frame需要转换到ENU坐标系下
	float degree=(final_index[0]+1)*sector_value1;
	degree-=180;
	degree=(int)(degree+360)%360;
    final_degree=fabs(degree-ang);
    final_indexNum=final_index[0];
	for(int j=0;j<final_index.size();j++)
	{
	    float degree=(final_index[j]+1)*sector_value1;
	    degree-=180;
	    degree=(int)(degree+360)%360;
		if(fabs(degree-ang)<final_degree)
		{
			//final_degree=(final_index[j]+1)*sector_value1;
			final_indexNum=final_index[j];
		}
	}
	printf("final_indexNum=%d\n",final_indexNum);
    printf("move_angle=%f\n",ang);
    //将选定的laser_frame下的valley转换到ENU坐标系下
	final_degree=(final_indexNum+1)*10;
	printf("finial degree in laser frame=%f\n",final_degree);
	final_degree-=180;
	final_degree=(int)(final_degree+360)%360;
	printf("finial degree in ENU= %f\n",final_degree);

	return final_degree;

}
//输入ENU坐标系下的当前航向
bool SectorMap::IsFrontSectorSafe(float ang)
{
	//ENU坐标系转换到laser坐标系
	ang+=180;
	ang=int(ang)%360;
	int index=(int)ang/10;
	if(map_cv[index-1]<0.1)
	{
		return true;
	}
	else
	{
		return false;
	}
	

}