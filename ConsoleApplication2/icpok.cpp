
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
#include <pcl/console/parse.h>  //pcl����̨����
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

	//icpʵ��	
	pcl::PointCloud<PointT>::Ptr cloud_pr(new pcl::PointCloud<PointT>);
	*cloud_pr = cloud;
	pcl::PointCloud<PointT>::Ptr cloud_hole_pr(new pcl::PointCloud<PointT>);
	*cloud_hole_pr = cloud_hole;
	pcl::IterativeClosestPoint<PointT, PointT> icp;

	icp.setInputSource(cloud_pr);  //ԭ����
	icp.setInputTarget(cloud_hole_pr);//Ŀ�����
	pcl::PointCloud<PointT> final_cloud;//��׼�����
										//icp ��������
	icp.setMaximumIterations(10000);  //��������������ʮ�ϰٶ����ܳ��֡�
	icp.setEuclideanFitnessEpsilon(0.5);//ǰ�����ε������Ĳ�ֵ,���ֵһ����Ϊ1e-6���߸�С.
	icp.setTransformationEpsilon(1e-10); //�ϴ�ת���뵱ǰת���Ĳ�ֵ��
	icp.setMaxCorrespondenceDistance(10); //�����ڴ˾���֮��ĵ㣬����������ƾ���ϴ����ֵҪ��Ĵ�һЩ��PCLĬ�Ͼ��뵥λ��m��������׼Ӱ��ϴ�
										   //������׼��ĵ�
	icp.align(final_cloud);
	//icpƥ����ת�����󼰵÷�
	cout << "has converged: " << icp.hasConverged() << endl
		<< "score: " << icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;

	//������������ɫ��Ϣ������鿴
	for (size_t i = 0; i < cloud.size(); ++i) {
		cloud.points[i].rgb = 2.3418052e-038;  //��
	}

	for (size_t i = 0; i < cloud_hole.size(); ++i) {
		cloud_hole.points[i].rgb = 3.5733111e-043;//��
	}

	for (size_t i = 0; i < final_cloud.size(); ++i) {
		final_cloud.points[i].rgb = 9.1476764e-041;//��
												   //final_cloud.points[i].rgb = 2.3418052e-038;
	}

	//show cloud

	cloud += cloud_hole;
	cloud += final_cloud;
	showCloud(cloud, "ƥ��Ч��");

	cout << "ICP �㷨ִ�����" << endl;
	system("pause");
	return (0);
}
