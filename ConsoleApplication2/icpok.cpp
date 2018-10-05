
#include<pcl/registration/icp.h>
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
#include <pcl/console/parse.h>  //pcl控制台解析
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>
obj2cloud(string obj_filename) {
	vtkSmartPointer<vtkPolyData> polydata;
	vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
	reader->SetFileName(obj_filename.c_str());
	reader->Update();
	polydata = reader->GetOutput();
	pcl::PointCloud<PointT> cloud;
	vtkPolyDataToPointCloud(polydata, cloud);
	return cloud;
}

void
showCloud(pcl::PointCloud<PointT> cloud, string windows_name) {
	pcl::PointCloud<PointT>::Ptr cloud_pr(new pcl::PointCloud<PointT>);
	*cloud_pr = cloud;

	pcl::visualization::PCLVisualizer viewer(windows_name);
	viewer.setCameraPosition(0, 0, -3.0, 0, -1, 0);
	viewer.addCoordinateSystem(0.3);

	viewer.addPointCloud(cloud_pr);

	while (!viewer.wasStopped())
		viewer.spinOnce(100);

}

void
addRgb(pcl::PointCloud<PointT>::Ptr cloud_pr, int rgb) {
	for (size_t i = 0; i < cloud_pr->size(); ++i) {
		cloud_pr->points[i].rgb = rgb;
	}
}
pcl::PointCloud<PointT> obj2cloud(string obj_filename);

void showCloud(pcl::PointCloud<PointT> cloud, string windows_name);

void addRgb(pcl::PointCloud<PointT>::Ptr cloud_pr, int rab);

int main() {
	string model_filename = "source.obj";
	string model_hole_filename = "force.obj";
	// Load the input file
	pcl::PointCloud<PointT> cloud = obj2cloud(model_filename);
	pcl::PointCloud<PointT> cloud_hole = obj2cloud(model_hole_filename);

	//icp实现	
	pcl::PointCloud<PointT>::Ptr cloud_pr(new pcl::PointCloud<PointT>);
	*cloud_pr = cloud;
	pcl::PointCloud<PointT>::Ptr cloud_hole_pr(new pcl::PointCloud<PointT>);
	*cloud_hole_pr = cloud_hole;
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	icp.setInputSource(cloud_pr);  //原点云
	icp.setInputTarget(cloud_hole_pr);//目标点云
	pcl::PointCloud<PointT> final_cloud;//配准后点云
										//icp 参数设置
	icp.setMaximumIterations(10000);  //最大迭代次数，几十上百都可能出现。
	icp.setEuclideanFitnessEpsilon(0.5);//前后两次迭代误差的差值,这个值一般设为1e-6或者更小.
	icp.setTransformationEpsilon(1e-10); //上次转换与当前转换的差值；
	icp.setMaxCorrespondenceDistance(10); //忽略在此距离之外的点，如果两个点云距离较大，这个值要设的大一些（PCL默认距离单位是m），对配准影响较大
										   //保存配准后的点
	icp.align(final_cloud);
	//icp匹配后的转换矩阵及得分
	cout << "has converged: " << icp.hasConverged() << endl
		<< "score: " << icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;

	//向点云里添加颜色信息，方便查看
	for (size_t i = 0; i < cloud.size(); ++i) {
		cloud.points[i].rgb = 2.3418052e-038;  //红
	}

	for (size_t i = 0; i < cloud_hole.size(); ++i) {
		cloud_hole.points[i].rgb = 3.5733111e-043;//绿
	}

	for (size_t i = 0; i < final_cloud.size(); ++i) {
		final_cloud.points[i].rgb = 9.1476764e-041;//蓝
												   //final_cloud.points[i].rgb = 2.3418052e-038;
	}

	//show cloud

	cloud += cloud_hole;
	cloud += final_cloud;
	showCloud(cloud, "匹配效果");

	cout << "ICP 算法执行完成" << endl;
	system("pause");
	return (0);
}
