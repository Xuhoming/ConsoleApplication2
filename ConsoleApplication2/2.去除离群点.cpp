#include<vtkRenderingCoreEnums.h>
#include<vtkRenderingCoreModule.h>
#include<vtkRenderingVolumeOpenGLModule.h>
#include<vtkRenderingVolumeOpenGLObjectFactory.h>
#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeType,vtkRenderingOpenGL) 
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)
#include <string>
#include <kinect.h>
#include <iostream>
#include <fstream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>  
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h> //ICP(iterative closest point)��׼
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/console/parse.h>  //pcl����̨����
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>  
//kd��
#include <pcl/kdtree/kdtree_flann.h>
//������ȡ
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//�ع�
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>

using namespace std;

string num2str(int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

//����ṹ�壬���ڴ������
struct PCD
{
	std::string f_name; //�ļ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //����ָ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormal;//�洢���Ƶķ��ߵ�ָ��
														   //���캯��
	PCD() : cloud(new pcl::PointCloud<pcl::PointXYZ>), cloudWithNormal(new pcl::PointCloud<pcl::PointNormal>) {}; //��ʼ��
};

int main()
{
	const int numberOfViews = 8;//��������
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //��������
	std::string prefix("cow");
	std::string extension(".pcd"); //��������ʼ��string���ͱ���extension����ʾ�ļ���׺��
								   // ͨ�������ļ�������ȡpcd�ļ�
	for (int i = 0; i < numberOfViews; i++) //�������е��ļ���
	{
		std::string fname = prefix + num2str(i) + extension;
		// ��ȡ���ƣ������浽models
		PCD m;
		m.f_name = fname;
		if (pcl::io::loadPCDFile(fname, *m.cloud) == -1) //* ����PCD��ʽ���ļ�������ļ������ڣ�����-1
		{
			cout << "Couldn't read file " + fname + "." << endl; //�ļ�������ʱ�����ش�����ֹ����
			return (-1);
		}
		data.push_back(m);
	}
	//ȥ����Ⱥ��
	for (int i = 0; i <= 1; ++i) {
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(data[i].cloud);
		outrem.setRadiusSearch(0.01);
		outrem.setMinNeighborsInRadius(10);
		outrem.filter(*data[i].cloud);
	}

	float yMax1 = -1000;
	for (int i = 0; i < data[0].cloud->size(); ++i)
	{
		if (data[0].cloud->points[i].y>yMax1)
			yMax1 = data[0].cloud->points[i].y;
	}
	cout << "yMax1:" << yMax1 << endl;

	float yMax2 = -1000;
	for (int i = 0; i < data[1].cloud->size(); ++i)
	{
		if (data[1].cloud->points[i].y>yMax2)
			yMax2 = data[1].cloud->points[i].y;
	}
	cout << "yMax2:" << yMax2 << endl;

	ofstream out("yMax.txt");
	if (out.is_open())
	{
		out << "yMax1:" << yMax1 << endl;
		out << "yMax2:" << yMax2 << endl;
		out.close();
	}
	//yMax1 = 0;yMax2 = 0;
	for (int i = 2; i < numberOfViews; ++i)
	{
		//cout << i << endl;
		
		//-----------------------ȥ���������--------------------------
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
		/*range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new	pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -0.4)));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new	pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.4)));
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new	pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 1.3)));*/
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new	pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, yMax1)));
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem(range_cond, false);
		condrem.setInputCloud(data[i].cloud);
		condrem.setKeepOrganized(false);
		condrem.filter(*data[i].cloud);

		//--------------------------------------------------------------

		//-----------------------ȥ����Ⱥ��------------------------
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(data[i].cloud);
		outrem.setRadiusSearch(0.01);
		outrem.setMinNeighborsInRadius(10);
		outrem.filter(*data[i].cloud);

		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		//sor.setInputCloud(data[i].cloud);
		//sor.setMeanK(10);
		//sor.setStddevMulThresh(1.0);
		//sor.filter(*data[i].cloud);
		//--------------------------------------------------------------

		// -------------------���㷨����----------------------
		//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //���������ƶ���ָ��
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ��ߵ�ָ��
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(data[i].cloud);
		n.setInputCloud(data[i].cloud);
		n.setSearchMethod(tree);
		n.setRadiusSearch(0.03);
		//n.setKSearch(25);
		n.compute(*normals); //���㷨�ߣ�����洢��normals��
		pcl::concatenateFields(*data[i].cloud, *normals, *data[i].cloudWithNormal);//�����ƺͷ��߷ŵ�һ��																		   // ---------------------------------------------------

		string fileName = "cow" + num2str(i-2) + "_withNormal.pcd";
		pcl::io::savePCDFile(fileName, *data[i].cloudWithNormal, false);//fu�޸�true��false
		cout << fileName << " has been saved." << endl;
		//cout << i << endl;
		// ��ʾ���ͼ
		/*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
		int v1;
		viewer->createViewPort(0.0, 0.0, 1.0, 1.0, v1);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->initCameraParameters();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_color1(data[i].cloudWithNormal, 255, 255, 255);
		viewer->addPointCloud(data[i].cloudWithNormal, cloud_color1, "cloud_color1", v1);
		viewer->addPointCloudNormals<pcl::PointNormal>(data[i].cloudWithNormal, 50, 0.05, "source_normals", v1); //��2��������ʾ���ٸ�����ʾһ����������3��������ʾ�������ȣ���λm
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "source_normals", v1);
		while (!viewer->wasStopped()) {
			viewer->spinOnce();
		}*/
	}


    system("pause");
	return 0;
}
