#include <iostream>
#include <string>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>  //pcl¿ØÖÆÌ¨½âÎö


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<PointT> obj2cloud(string obj_filename);

void showCloud(pcl::PointCloud<PointT> cloud, string windows_name);

void addRgb(pcl::PointCloud<PointT>::Ptr cloud_pr, int rab);
