#include<vtkRenderingCoreEnums.h>
#include<vtkRenderingCoreModule.h>
#include<vtkRenderingVolumeOpenGLModule.h>
#include<vtkRenderingVolumeOpenGLObjectFactory.h>
#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeType,vtkRenderingOpenGL) 
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)
#include <string>
#include <kinect.h>
#include <iostream>
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
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
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
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
int main() {
	//���ص����ļ�
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointNormal>);
	pcl::io::loadPCDFile("source.pcd", *cloud1);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::io::loadPCDFile("force.pcd", *cloud2);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	*cloud = *cloud1;
	*cloud += *cloud2;
	pcl::io::savePCDFileASCII("END.pcd", *cloud);

}