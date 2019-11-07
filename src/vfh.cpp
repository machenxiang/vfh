#include"vfh.h"

#include<cmath>
#include<algorithm>
//
void SectorMap::SetUavPosition(Point2D uav)
{
    Uavp.x=uav.x;
    Uavp.y=uav.y;
}

void SectorMap::ComputeCV(vector<float> r)
{
    float dist[360]={0};
    ranges.clear();
    map_cv.clear();
    for(int i=0;i<r.size();i++)
    {
        if(!std::isnan(r[i])&&!std::isinf(r[i]))
        {
            float scan_distance=r[i];
            int sector_index=std::floor((i*angle_resolution)/sector_value);
            if(scan_distance>scan_distance_max||scan_distance<scan_distance_min)
            {
                scan_distance=0;
            }
            else
            {
                //激光雷达的扫描到的安全距离越大越安全
                scan_distance=scan_distance_max-scan_distance;
            }
            dist[sector_index]+=scan_distance;
            
        }
        //保存激光雷达原始数据
        ranges.push_back(r[i]);
    }
    //存入每个扇区的cv值
    for(int j=0;j<(int)360/sector_value;j++)
    {
        map_cv.push_back(dist[j]);
    }
}

bool SectorMap::IsFrontSafety(float ang)
{
    //传进来弧度制先转换为角度值
    ang=int(ang*(180/PI));
    ang+=360;
    ang=int(ang)%360;
    //坐标系变换
    ang+=180;
	ang=int(ang)%360;
    //转换到激光雷达坐标系下判断是否安全
    //将非10的整数倍的ang映射到map_cv扇区
    int index=std::floor(ang/10);
    if((map_cv[index-2]<0.1)&&(map_cv[index-1]<0.1)&&(map_cv[index]<0.1)&&(map_cv[index+1]<0.1)&&(map_cv[index+2]<0.1))
    {
       //ROS_INFO("front safety");
        return true;
    }
    else
    {
        //ROS_INFO("front dangerous");
        return false;
    }
    
}

float SectorMap::CalculDirection(Point2D goal)
{
    float direction=atan2((goal.y-Uavp.y),(goal.x-Uavp.x));

    return direction;
    
}
float SectorMap::SelectNewDir(float ang)
{
    //传进来弧度制先转换为角度值
    ang=int(ang*(180/PI));
    ang+=360;
    ang=int(ang)%360;
    ROS_INFO("ang in ENU %f \n",ang);
    //坐标系变换
    ang+=180;
	ang=int(ang)%360;
    ROS_INFO("ang in laser_frame %f \n",ang);
    //选择距离目标最近的安全扇区
    vector<int> SafeValleyIndex;
    for(int j=0;j<int(360/sector_value);j++)
    {

        int h=j-2;
        h+=36;
        h=int(h)%36;

        int i=j-1;
        i+=36;
        i=int(i)%36;

        int k=k+1;
        i+=36;
        k=int(k)%36;

        int l=l+1;
        l+=36;
        l=int(l)%36;    

        if(map_cv[h]<0.1&&map_cv[i]<0.1&&map_cv[j]<0.1&&map_cv[k]<0.1&&map_cv[l]<0.1)
        {
            SafeValleyIndex.push_back(j);
        }
    }
    float min_dif=fabs((SafeValleyIndex[0])*10-ang);
    int Candidate_index=SafeValleyIndex[0];
    for(int i=1;i<SafeValleyIndex.size();i++)
    {
        if(fabs((SafeValleyIndex[i])*10-ang)<min_dif)
        {
            min_dif=fabs((SafeValleyIndex[i])*10-ang);
            Candidate_index=SafeValleyIndex[i];
        }
    }
    ROS_INFO("candidate index %d min_dif %f \n,",Candidate_index,min_dif);
    ang=Candidate_index*10;
    ang-=180;
    ang+=360;
    ang=int(ang)%360;
    ROS_INFO("ang %f consider front safety\n",ang);
    ang=ang*(PI/180);

    return ang;//在ENU坐标系下的角度，都用弧度制
}

