#include "ndt_map/ndt_map.h"
#include "ndt_map/lazy_grid.h"
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <limits.h>
#include <unistd.h>
#include "ndt_map/stopwatch.h"
#include "ndt_map/costmap.h"
using namespace std;

//self-time test
bool loadCloud(std::string &filename,pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  std::cout << "Loading file " << filename.c_str() << std::endl;
  //read cloud
  if (pcl::io::loadPCDFile(filename, *cloud))
  {
    std::cout << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return false;
  }
  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;
  return true;
}

int main()
{
    std::string cloud_path("/home/daysun/rros/fullSys.pcd"); //site125 //freiburg2_16
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (!loadCloud(cloud_path,cloud))
      return -1;

    double time1 = stopwatch();
    perception_oru::NDTMap nd(new perception_oru::LazyGrid(0.5));
    Eigen::Affine3d T;
    T.setIdentity();
    nd.addPointCloud(T.translation(),*cloud);
    nd.computeNDTCells();

    std::vector<perception_oru::NDTCell*> ndts;
    ndts = nd.getAllCells();
//    cout<<ndts.size()<<endl;
    vector<perception_oru::NDTCell*>::iterator it = ndts.begin();
    while(it != ndts.end()){
        (*it)->computeGaussian();
        it++;
    }

    double time2 = stopwatch();
    cout<<"division time "<<(time2-time1)<<" s\n";


   double robot_height=1;
    double time3 = stopwatch();
    perception_oru::NDTCostmap cmap(&nd);
    cmap.processMap(robot_height, -1);
    double time4 = stopwatch();
    cout<<"process map done\n";
    cout<<"costmap time "<<(time4-time3)<<" s\n";

//    nd.writeToJFF("mapsfullSys.jff");
    cout<<"count the map \n";
}
