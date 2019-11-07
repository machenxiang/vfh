#ifndef VFH_H
#define VFH_H

#include<ros/ros.h>
#include<vector>
using namespace std;

#define PI 3.1415926

struct Point2D
{
    float x;
    float y;
};

class SectorMap
{
public:
	SectorMap()
	{
		scan_distance_max = 2.1;
		scan_distance_min = 0.1;
		angle_resolution = 1.0;

		sector_value = 10;

		Uavp.x = 0;
		Uavp.y = 0;
	}
	virtual ~SectorMap()
	{

	}

	void ComputeCV(vector<float> r);//计算扇区cv值
	float CalculDirection(Point2D goal);//返回弧度制用于计算速度，发送角度值会震荡
	bool IsFrontSafety(float ang);//判断前进方向扇区是否安全,前进方向30°夹角cv值小于0.1视为安全
	void SetUavPosition(Point2D uav);//用于位置更新
	float SelectNewDir(float ang);//前方存在障碍物用于选择新的方向

	float scan_distance_max;
	float scan_distance_min;
	float angle_resolution;

	float sector_value;

	Point2D Uavp;
	vector<float> map_cv;
	vector<double> ranges;
};



#endif