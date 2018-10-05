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


using namespace std;

//const float yMax = -0.218759;

string num2str(int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

// Returns the rotation matrix around a vector  placed at a point , rotate by angle t
Eigen::Matrix4f rot_mat(const Eigen::Vector3f& point, const Eigen::Vector3f& vector, const float t)
{
	float u = vector(0);
	float v = vector(1);
	float w = vector(2);
	float a = point(0);
	float b = point(1);
	float c = point(2);

	Eigen::Matrix4f matrix;
	matrix << u*u + (v*v + w*w)*cos(t), u*v*(1 - cos(t)) - w*sin(t), u*w*(1 - cos(t)) + v*sin(t), (a*(v*v + w*w) - u*(b*v + c*w))*(1 - cos(t)) + (b*w - c*v)*sin(t),
		u*v*(1 - cos(t)) + w*sin(t), v*v + (u*u + w*w)*cos(t), v*w*(1 - cos(t)) - u*sin(t), (b*(u*u + w*w) - v*(a*u + c*w))*(1 - cos(t)) + (c*u - a*w)*sin(t),
		u*w*(1 - cos(t)) - v*sin(t), v*w*(1 - cos(t)) + u*sin(t), w*w + (u*u + v*v)*cos(t), (c*(u*u + v*v) - w*(a*u + b*v))*(1 - cos(t)) + (a*v - b*u)*sin(t),
		0, 0, 0, 1;
	return matrix;
}

//����ṹ�壬���ڴ������
struct PCD
{
	std::string f_name; //�ļ���
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormal;//�洢���Ƶķ��ߵ�ָ��
	Eigen::Vector3f mass;//�洢���Ƶ�����
						 //���캯��
	PCD() : cloudWithNormal(new pcl::PointCloud<pcl::PointNormal>), mass(Eigen::Vector3f::Zero()) {}; //��ʼ��
};

void UpdatePCDMass(PCD& m)
{
	m.mass.Zero();
	for (int j = 0; j < m.cloudWithNormal->size(); ++j)
	{
		m.mass(0) += m.cloudWithNormal->points[j].x;
		m.mass(1) += m.cloudWithNormal->points[j].y;
		m.mass(2) += m.cloudWithNormal->points[j].z;
	}
	m.mass /= m.cloudWithNormal->size();
}

struct AABB
{
	Eigen::Vector3f center;
	Eigen::Vector3f min;
	Eigen::Vector3f max;
};

AABB computerAABB(pcl::PointCloud<pcl::PointNormal>::Ptr p)
{
	AABB aabb;
	aabb.min(0) = +10000;
	aabb.min(1) = +10000;
	aabb.min(2) = +10000;
	aabb.max(0) = -10000;
	aabb.max(1) = -10000;
	aabb.max(2) = -10000;
	for (int i = 0; i < p->size(); ++i)
	{
		if (p->points[i].x < aabb.min(0))
			aabb.min(0) = p->points[i].x;
		if (p->points[i].y < aabb.min(1))
			aabb.min(1) = p->points[i].y;
		if (p->points[i].z < aabb.min(2))
			aabb.min(2) = p->points[i].z;

		if (p->points[i].x > aabb.max(0))
			aabb.max(0) = p->points[i].x;
		if (p->points[i].y > aabb.max(1))
			aabb.max(1) = p->points[i].y;
		if (p->points[i].z > aabb.max(2))
			aabb.max(2) = p->points[i].z;
	}
	aabb.center = 0.5f*(aabb.max + aabb.min);
	return aabb;
}

int main()
{
	const int numberOfViews = 6;//��������
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data; //��������
	std::string prefix("cow");
	std::string extension("_withNormal.pcd"); //��������ʼ��string���ͱ���extension����ʾ�ļ���׺��
											  // ͨ�������ļ�������ȡpcd�ļ�
	for (int i = 0; i < numberOfViews; i++) //�������е��ļ���
	{
		std::string fname = prefix + num2str(i) + extension;
		// ��ȡ���ƣ������浽models
		PCD m;
		m.f_name = fname;
		if (pcl::io::loadPCDFile(fname, *m.cloudWithNormal) == -1) //* ����PCD��ʽ���ļ�������ļ������ڣ�����-1
		{
			cout << "Couldn't read file " + fname + "." << endl; //�ļ�������ʱ�����ش�����ֹ����
			return (-1);
		}
		for (int j = 0; j < m.cloudWithNormal->size(); ++j)
		{
			m.mass(0) += m.cloudWithNormal->points[j].x;
			m.mass(1) += m.cloudWithNormal->points[j].y;
			m.mass(2) += m.cloudWithNormal->points[j].z;
		}
		m.mass /= m.cloudWithNormal->size();
		data.push_back(m);
	}

	//-----------------------ȥ����Ⱥ��------------------------
	pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
	outrem.setRadiusSearch(0.01);
	outrem.setMinNeighborsInRadius(15);
	for (int i = 0; i < numberOfViews; ++i)
	{
		outrem.setInputCloud(data[i].cloudWithNormal);
		outrem.filter(*data[i].cloudWithNormal);
		UpdatePCDMass(data[i]);
	}
	//--------------------------------------------------------------

	Eigen::Vector3f totalMass1 = Eigen::Vector3f::Zero();//��¼���е��Ƶ�����
	int totalNumberOfPoints1 = 0;//��¼���е��Ƶĵ����
	for (int i = 0; i < 3; ++i)
	{
		totalNumberOfPoints1 += data[i].cloudWithNormal->size();
		totalMass1 += data[i].mass * data[i].cloudWithNormal->size();
	}
	totalMass1 /= totalNumberOfPoints1;

	Eigen::Vector3f totalMass2 = Eigen::Vector3f::Zero();//��¼���е��Ƶ�����
	int totalNumberOfPoints2 = 0;//��¼���е��Ƶĵ����
	for (int i = 3; i < 6; ++i)
	{
		totalNumberOfPoints2 += data[i].cloudWithNormal->size();
		totalMass2 += data[i].mass * data[i].cloudWithNormal->size();
	}
	totalMass2 /= totalNumberOfPoints2;

	Eigen::Vector3f upVector(0, 1.0, 0);
	Eigen::Matrix4f rotationMatrix = rot_mat(totalMass1, upVector, -M_PI / 3);
	pcl::transformPointCloudWithNormals(*data[1].cloudWithNormal, *data[1].cloudWithNormal, rotationMatrix);
	UpdatePCDMass(data[1]);

	rotationMatrix = rot_mat(totalMass1, upVector, M_PI / 3);
	pcl::transformPointCloudWithNormals(*data[2].cloudWithNormal, *data[2].cloudWithNormal, rotationMatrix);
	UpdatePCDMass(data[2]);

	rotationMatrix = rot_mat(totalMass2, upVector, M_PI / 3);
	pcl::transformPointCloudWithNormals(*data[4].cloudWithNormal, *data[4].cloudWithNormal, rotationMatrix);
	UpdatePCDMass(data[4]);

	rotationMatrix = rot_mat(totalMass2, upVector, -M_PI / 3);
	pcl::transformPointCloudWithNormals(*data[5].cloudWithNormal, *data[5].cloudWithNormal, rotationMatrix);
	UpdatePCDMass(data[5]);


	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icpWithNormals;
	icpWithNormals.setMaxCorrespondenceDistance(0.5);
	icpWithNormals.setMaximumIterations(100);
	icpWithNormals.setTransformationEpsilon(1e-10);
	icpWithNormals.setEuclideanFitnessEpsilon(0.01);

	icpWithNormals.setInputCloud(data[1].cloudWithNormal);
	icpWithNormals.setInputTarget(data[0].cloudWithNormal);
	icpWithNormals.align(*data[1].cloudWithNormal);

	icpWithNormals.setInputCloud(data[2].cloudWithNormal);
	icpWithNormals.setInputTarget(data[0].cloudWithNormal);
	icpWithNormals.align(*data[2].cloudWithNormal);

	icpWithNormals.setInputCloud(data[4].cloudWithNormal);
	icpWithNormals.setInputTarget(data[3].cloudWithNormal);
	icpWithNormals.align(*data[4].cloudWithNormal);

	icpWithNormals.setInputCloud(data[5].cloudWithNormal);
	icpWithNormals.setInputTarget(data[3].cloudWithNormal);
	icpWithNormals.align(*data[5].cloudWithNormal);


	pcl::PointCloud<pcl::PointNormal>::Ptr Front(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr Back(new pcl::PointCloud<pcl::PointNormal>);
	// ----------------------------------------------------
	*Front += *data[0].cloudWithNormal;
	*Front += *data[1].cloudWithNormal;
	*Front += *data[2].cloudWithNormal;
	UpdatePCDMass(data[1]);
	UpdatePCDMass(data[2]);

	*Back += *data[3].cloudWithNormal;
	*Back += *data[4].cloudWithNormal;
	*Back += *data[5].cloudWithNormal;
	UpdatePCDMass(data[4]);
	UpdatePCDMass(data[5]);




	// ----------------------------------------------------

	//---------------ǰ����׼------------------------------
	Eigen::Vector3f mass = Eigen::Vector3f::Zero();
	for (int i = 0; i < Back->points.size(); ++i) {
		mass(0) += Back->points[i].x;
		mass(1) += Back->points[i].y;
		mass(2) += Back->points[i].z;
	}
	mass /= Back->points.size();
	rotationMatrix = rot_mat(mass, upVector, M_PI);
	pcl::transformPointCloudWithNormals(*Back, *Back, rotationMatrix);

	Eigen::Vector3f plane_left(-1, 0, 0);
	Eigen::Vector3f plane_right(1, 0, 0);
	





	AABB BackAABB = computerAABB(Back);
	cout << "BackAABB's center:\n" << BackAABB.center << endl;
	cout << "BackAABB's Z Length:\n" << BackAABB.max(2) - BackAABB.min(2) << endl;
	AABB FrontAABB = computerAABB(Front);
	cout << "FrontAABB's center:\n" << FrontAABB.center << endl;
	cout << "FrontAABB's Z Length:\n" << FrontAABB.max(2) - FrontAABB.min(2) << endl;
	Eigen::Vector3f diff = FrontAABB.center - BackAABB.center;
	Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();
	translationMatrix(0, 3) = diff(0);
	translationMatrix(1, 3) = diff(1);
	translationMatrix(2, 3) = FrontAABB.max(2) - BackAABB.min(2) - (FrontAABB.max(2) - FrontAABB.min(2))*0.8;
	pcl::transformPointCloudWithNormals(*Back, *Back, translationMatrix);





	cout << "��������׼" << endl;
	int iteration = 100;
	float cos_angle = cos(M_PI * 10 / 180);
	for (int iter = 0; iter < iteration; ++iter)
	{
		pcl::IndicesPtr source_indices(new std::vector<int>());
		for (int i = 0; i < Back->points.size(); ++i) {
			//if (Back->points[i].y>yMax)
			//	continue;
			Eigen::Vector3f n = Back->points[i].getNormalVector3fMap();
			n.normalize();
			if (n.transpose() * plane_left > cos_angle) {
				source_indices->push_back(i);
				continue;
			}
			if (n.transpose() * plane_right > cos_angle) {
				source_indices->push_back(i);
			}
		}
		cout << "Source Indices: " << source_indices->size() << endl;

		pcl::IndicesPtr target_indices(new std::vector<int>());
		for (int i = 0; i < Front->points.size(); ++i) {
			//if (Front->points[i].y>yMax)
				//continue;
			Eigen::Vector3f n = Front->points[i].getNormalVector3fMap();
			n.normalize();
			if (n.transpose() * plane_left > cos_angle) {
				target_indices->push_back(i);
				continue;
			}
			if (n.transpose() * plane_right > cos_angle) {
				target_indices->push_back(i);
			}
		}
		cout << "Target Indices: " << target_indices->size() << endl;

		pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> correst;
		correst.setInputCloud(Back);
		correst.setSourceNormals(Back);
		correst.setInputTarget(Front);
		correst.setIndicesSource(source_indices);
		correst.setIndicesTarget(target_indices);
		correst.setKSearch(15);
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
		correst.determineReciprocalCorrespondences(*all_correspondences);
		//cout << "Correspondences (Before) : " << all_correspondences->size() << "\n";

		pcl::registration::CorrespondenceRejectorSurfaceNormal rejector;
		rejector.initializeDataContainer<pcl::PointNormal, pcl::PointNormal>();
		rejector.setInputSource<pcl::PointNormal>(Back);
		rejector.setInputTarget<pcl::PointNormal>(Front);
		rejector.setInputNormals<pcl::PointNormal, pcl::PointNormal>(Back);
		rejector.setTargetNormals<pcl::PointNormal, pcl::PointNormal>(Front);
		rejector.setInputCorrespondences(all_correspondences);
		rejector.setThreshold(M_PI * 10 / 180);
		pcl::CorrespondencesPtr correspondences_after_rejector(new pcl::Correspondences);
		rejector.getCorrespondences(*correspondences_after_rejector);
		//cout << "Correspondences (After) : " << correspondences_after_rejector->size() << "\n";


		Eigen::Matrix4f transformation;

		//pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> trans_est_svd;
		//trans_est_svd.estimateRigidTransformation(*Back, *Front, *correspondences_after_rejector, transformation);

		pcl::registration::TransformationEstimationLM<pcl::PointNormal, pcl::PointNormal> trans_est_lm;
		trans_est_lm.estimateRigidTransformation(*Back, *Front, *correspondences_after_rejector, transformation);

		//pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> trans_est_PointToPlane;
		//trans_est_PointToPlane.estimateRigidTransformation(*Back, *Front, *correspondences_after_rejector, transformation);

		pcl::transformPointCloudWithNormals(*Back, *Back, transformation);
		cout << "Iteration: " << iter << endl;
		//cout << "Matrix " << iter << ":\n" << transformation << endl;
		if (transformation.isIdentity())
			break;
	}
	pcl::io::savePCDFile("text16.txt", *Back, false);
	cout << "��������׼" << endl;
	float cos_angle2 = cos(M_PI * 15/ 180);
	for (int iter = 95; iter < iteration; ++iter)
	{
		pcl::IndicesPtr source_indices(new std::vector<int>());
		for (int i = 0; i < Back->points.size(); ++i) {
			//if (Back->points[i].y>yMax)
			//	continue;
			Eigen::Vector3f n = Back->points[i].getNormalVector3fMap();
			n.normalize();
			if (n.transpose() * upVector < cos_angle2) {
				source_indices->push_back(i);
				continue;
			}
		}
		cout << "Source Indices: " << source_indices->size() << endl;


		pcl::IndicesPtr target_indices(new std::vector<int>());
		for (int i = 0; i < Front->points.size(); ++i) {
			//if (Front->points[i].y>yMax)
			//	continue;
			Eigen::Vector3f n = Front->points[i].getNormalVector3fMap();
			n.normalize();
			if (n.transpose() * upVector < cos_angle2) {
				target_indices->push_back(i);
				continue;
		}
		}
		cout << "Target Indices: " << target_indices->size() << endl;

		pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> correst;
		correst.setInputCloud(Back);
		correst.setSourceNormals(Back);
		correst.setInputTarget(Front);
		correst.setIndicesSource(source_indices);
		correst.setIndicesTarget(target_indices);
		correst.setKSearch(15);
		pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
		correst.determineReciprocalCorrespondences(*all_correspondences);
		//cout << "Correspondences (Before) : " << all_correspondences->size() << "\n";

		pcl::registration::CorrespondenceRejectorSurfaceNormal rejector;
		rejector.initializeDataContainer<pcl::PointNormal, pcl::PointNormal>();
		rejector.setInputSource<pcl::PointNormal>(Back);
		rejector.setInputTarget<pcl::PointNormal>(Front);
		rejector.setInputNormals<pcl::PointNormal, pcl::PointNormal>(Back);
		rejector.setTargetNormals<pcl::PointNormal, pcl::PointNormal>(Front);
		rejector.setInputCorrespondences(all_correspondences);
		rejector.setThreshold(M_PI * 15/ 180);
		pcl::CorrespondencesPtr correspondences_after_rejector(new pcl::Correspondences);
		rejector.getCorrespondences(*correspondences_after_rejector);
		//cout << "Correspondences (After) : " << correspondences_after_rejector->size() << "\n";

		Eigen::Matrix4f transformation;

		pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> trans_est_svd;
		trans_est_svd.estimateRigidTransformation(*Back, *Front, *correspondences_after_rejector, transformation);

		//pcl::registration::TransformationEstimationLM<pcl::PointNormal, pcl::PointNormal> trans_est_lm;
		//trans_est_lm.estimateRigidTransformation(*Back, *Front, *correspondences_after_rejector, transformation);

		//pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> trans_est_PointToPlane;
		//trans_est_PointToPlane.estimateRigidTransformation(*Back, *Front, *correspondences_after_rejector, transformation);

		pcl::transformPointCloudWithNormals(*Back, *Back, transformation);
		cout << "Iteration: " << iter << endl;
		//cout << "Matrix " << iter << ":\n" << transformation << endl;
		if (transformation.isIdentity())
			break;
	}

	BackAABB = computerAABB(Back);
	cout << "BackAABB's center:\n" << BackAABB.center << endl;
	cout << "BackAABB's Z Length:\n" << BackAABB.max(2) - BackAABB.min(2) << endl;
	FrontAABB = computerAABB(Front);
	cout << "FrontAABB's center:\n" << FrontAABB.center << endl;
	cout << "FrontAABB's Z Length:\n" << FrontAABB.max(2) - FrontAABB.min(2) << endl;

	cout << "Z Length:\n" << FrontAABB.max(2) - BackAABB.min(2) << endl;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointNormal>);
	*cloud = *Back;
	*cloud += *Front;

	// -------------------�����ؽ�----------------------
	//����������
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud);
	////����Poisson���󣬲����ò���
	//pcl::Poisson<pcl::PointNormal> pn;
	//pn.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	//pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
	//pn.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
	//pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
	//pn.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	//pn.setOutputPolygons(true); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	//pn.setSamplesPerNode(3.0); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
	//pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	//pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������
	////pn.setIndices();
	////���������������������
	//pn.setSearchMethod(tree2);
	//pn.setInputCloud(cloud);
	////����������������ڴ洢���
	//pcl::PolygonMesh mesh;
	////ִ���ع�
	//pn.performReconstruction(mesh);
	// ---------------------------------------------------

	pcl::io::savePLYFile("test cow.ply", *cloud, true);

	// ��ʾ���ͼ
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
	int v1; //������������v1��v2������v1������ʾ��ʼλ�ã�v2������ʾ��׼����
	int v2;
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //�ĸ����ڲ����ֱ��Ӧx_min,y_min,x_max.y_max.
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0, 0, 0);
	//viewer->addPolygonMesh(mesh, "mesh2", v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_color4(Front, 0, 255, 0);
	viewer->addPointCloud(Front, cloud_color4, "cloud_color4", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_color5(Back, 255, 0, 0);
	viewer->addPointCloud(Back, cloud_color5, "cloud_color5", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_color6(Front, 255, 255, 255);
	viewer->addPointCloud(Front, cloud_color6, "cloud_color6", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_color7(Back, 255, 255, 255);
	viewer->addPointCloud(Back, cloud_color7, "cloud_color7", v2);

	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce();
	}

	std::system("pause");
	return 0;
}