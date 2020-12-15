#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseArray.h"

using namespace std;
ros::Publisher mapPub;
nav_msgs::OccupancyGrid rosMap;
std::vector<int> pMap;

std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

//设置地图参数
void SetMapParams(void )
{
   //地图大小、分辨率
   mapParams.width = 900;
   mapParams.height = 900;     //单位栅格个数
   mapParams.resolution = 0.04;//0.04m 1/0.04=25个栅格
   //假设free=-1、occ=2
   mapParams.log_free = -1;
   mapParams.log_occ = 2;
   //
   mapParams.origin_x = 0.0;
   mapParams.origin_y = 0.0;

   //地图的原点，即是机器人默认位置
   mapParams.offset_x = 500;
   mapParams.offset_y = 400;  //单位栅格个数
   //为地图指针申请空间
   
   pMap.resize(mapParams.width*mapParams.height);
   //pMap = new int[mapParams.width*mapParams.height];

   //每一个栅格代表的值，初始化为50
   for(int i = 0; i < mapParams.width * mapParams.height;i++)
        pMap[i] = 50;
}

//从世界坐标系转换到栅格坐标系，主要是存在一个分辨率
//比如resolution = 0.04，世界坐标系下，单位1在栅格坐标系可以表示1/resolution=25个栅格
//目的：将机器人的实际位置，在900x900的栅格地图中找到对应的栅格序号，返回GridIndex对象
GridIndex ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;
    //ceil()向上取整函数
    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

//从栅格序号，转化到数组序号；因为栅格最终是按照顺序（width从小到大，height从低到高）依次存储到动态数组中的
int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
}

//判断index是否有效
//目的：判断该栅格序号是否在设定栅格地图大小范围内
bool isValidGridIndex(GridIndex index)
{
    if(index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

//销毁地图指针
/*void DestoryMap()
{
    if(pMap != NULL)
        delete pMap;
}*/

//占据栅格地图构建算法
//输入地图点位姿数据和相机位姿数据
//目的：通过遍历所有帧数据，为pMap[]中的每个穿过的空闲栅格或者击中栅格赋新值，中间有个计算方法，也就是占用栅格地图构建的理论实现
void OccupanyMapping(std::vector<double>& mapPointPoses,double cam_x,double cam_z,int &point_count)
{
  
    //获取该帧下的相机位姿
    //Eigen::Vector2d cameraPose = cameraPoses[frame];
    //获取该帧下的相机位姿的栅格序号
    GridIndex cameraIndex = ConvertWorld2GridIndex(cam_x,cam_z);
    
 
    //判断该帧相机位姿的栅格序号，是否在自己设定的栅格地图范围内
    if(isValidGridIndex(cameraIndex) == false) return;

    //遍历该帧地图点位姿数据所有x和y
    for(int id = 0; id < point_count; id += 2)
    {

      //得到当前这个地图点的x和y，即在世界坐标系下的位置
      double world_x = mapPointPoses[id];
      double world_z = mapPointPoses[id+1];

      //将该地图点在世界坐标系下的位置，转化为栅格序号
      GridIndex mapIndex = ConvertWorld2GridIndex(world_x,world_z);

      //判断该地图点的栅格序号，是否在自己设定的栅格地图900x900范围内，如果不在则跳过
      if(isValidGridIndex(mapIndex) == false)continue;

      //从相机的栅格序号到该地图点的栅格序号划线
      //目的：找到两点之间途径的空闲栅格，将栅格序号存入std::vector<GridIndex>中
      std::vector<GridIndex> freeIndex = TraceLine(cameraIndex.x,cameraIndex.y,mapIndex.x,mapIndex.y);

      //遍历该扫描激光点通过的所有空闲栅格
      for(int k = 0; k < freeIndex.size();k++)
      {
        GridIndex tmpIndex = freeIndex[k];
        //将空闲栅格的栅格序号，转化到数组序号,该数组用于存储每一个栅格的数据
        int linearIndex = GridIndexToLinearIndex(tmpIndex);
        //取出该栅格代表的数据
        int data = pMap[linearIndex];
        //根据栅格空闲规则，执行data += mapParams.log_free;
        if(data > 0)//默认data=50
          data += mapParams.log_free;//log_free=-1，data将变小
        else
          data = 0;
        //给该空闲栅格赋新值，最小为0
        pMap[linearIndex] = data;

      }

      //更新该地图点集中的栅格，
      int tmpIndex = GridIndexToLinearIndex(mapIndex);
      int data = pMap[tmpIndex];
      //根据栅格击中规则，执行data += mapParams.log_occ;
      if(data < 100)//默认data=50
        data += mapParams.log_occ;//log_occ=2，data将变大
      else
        data = 100;
      //给击中的栅格赋新值，最大100
      pMap[tmpIndex] = data;
      //到这里，对一个位姿下的一个地图点数据经过的空闲栅格和击中栅格的pMap进行了重新赋值
    }
    //到这里，对一个位姿下的一帧所有地图点数据经过的空闲栅格和击中栅格进行了重新赋值
}

//发布地图．
void PublishMap(ros::Publisher& map_pub)
{
   
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
    {

       if(pMap[i] == 50)    //未知栅格
       {
           rosMap.data[i] = -1.0;
       }
       else if(pMap[i] < 50)//空闲栅格
       {
            rosMap.data[i] =10;   //gmapping    方式
          // rosMap.data[i] = pMap[i];//cartographer方式
       }
       else if(pMap[i] > 50)//击中栅格
       {
            rosMap.data[i] =80;
          // rosMap.data[i] = pMap[i];
       }
    }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}


void callback(const geometry_msgs::PoseArray& kf_and_pts)
{
     std::vector<double> mapPointPoses;
     int size=(int)kf_and_pts.poses[1].position.x;
     double cam_x=kf_and_pts.poses[0].position.x*3;
     double cam_z=kf_and_pts.poses[0].position.z*3;
     for(int i=0;i<size;i++)
     {
       double point_x=kf_and_pts.poses[2+i].position.x;
       double point_z=kf_and_pts.poses[2+i].position.z;
       point_x*=3;
       point_z*=3;
       mapPointPoses.push_back(point_x);
       mapPointPoses.push_back(point_z);
     }
     OccupanyMapping(mapPointPoses,cam_x,cam_z,size);
     
     PublishMap(mapPub);
     
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OccupanyMapping");

  ros::NodeHandle nodeHandler;

  mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map",1,true);
 
  //设置地图信息
  SetMapParams();

  //初始化栅格地图
  

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

 //新建一个sub，Callback是回调函数名
    ros::Subscriber sub = nodeHandler.subscribe("pts_and_pose", 1000, callback);
    ros::spin();
  
}




