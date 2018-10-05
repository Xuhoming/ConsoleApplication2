//

//#include <vtkAutoInit.h> 
//VTK_MODULE_INIT(vtkRenderingOpenGL)

#include<vtkRenderingCoreEnums.h>
#include<vtkRenderingCoreModule.h>
#include<vtkRenderingVolumeOpenGLModule.h>
#include<vtkRenderingVolumeOpenGLObjectFactory.h>
#define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeType,vtkRenderingOpenGL) 
#define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)
 

#include <Windows.h>
#include <iostream>
#include <kinect.h>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>  
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h> //ICP(iterative closest point)��׼
#include <pcl/console/parse.h>  //pcl����̨����
//kd��
#include <pcl/kdtree/kdtree_flann.h>
//������ȡ
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//�ع�
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

#include <boost/thread/thread.hpp>
#include <string.h>
using namespace cv;
using namespace std;

typedef pcl::PointXYZ  MyPointDataType;

// ��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

string num2str(int i)
{
	stringstream ss;
	ss << i;
	return ss.str();
}

int	main	()
{
	// ��ȡKinect�豸
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader;
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// ��ȡ������Դ����ȡ��  
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				//FrameSourceTypes::FrameSourceTypes_Color |
				//FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	UINT16 *depthData = new UINT16[424 * 512];//���ڴ洢���ͼ����
	Mat i_rgb(1080, 1920, CV_8UC4);
	Mat i_depthWrite(424, 512, CV_16UC1);
	UINT nColorBufferSize = 1920 * 1080 * 4;

	// ��������֡������
	IDepthFrameReference* m_pDepthFrameReference = nullptr;
	IColorFrameReference* m_pColorFrameReference = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;
	IColorFrame* m_pColorFrame = nullptr;
	IMultiSourceFrame* m_pMultiFrame = nullptr;

	ICoordinateMapper* m_pCoordinateMapper = nullptr;


	int count = 0;
	while (count <= 30)
	{

		//Sleep(5000);
		system("pause");
		while (true)
		{
			// ��ȡ�µ�һ����Դ����֡
			hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
			if (FAILED(hr) || !m_pMultiFrame)
			{
				continue;
			}
			break;
		}

		// �Ӷ�Դ����֡�з������ɫ���ݣ��������
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		//if (SUCCEEDED(hr))
		//	hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		//if (SUCCEEDED(hr))
		//	hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);

		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

		//if (SUCCEEDED(hr))
		//	hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, i_rgb.data, ColorImageFormat::ColorImageFormat_Bgra);

		// ������ر���
		pcl::PointCloud<MyPointDataType>::Ptr cloud(new pcl::PointCloud<MyPointDataType>);
		pcl::PointCloud<MyPointDataType>::Ptr cloud_filtered(new pcl::PointCloud<MyPointDataType>);
		//��ʼ����������PCD�ļ�ͷ
		cloud->width = 512 * 424;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);

		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			/*hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depthWrite.data));
			imwrite("depth_" + num2str(count) + ".png", i_depthWrite);*/
			CameraSpacePoint* m_pCameraCoordinates = new CameraSpacePoint[512 * 424];
			hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, depthData, 512 * 424, m_pCameraCoordinates);
			//ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];
			//hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);
			for (int i = 0; i < 512 * 424; i++)
			{
				//------д��RGB------
				/*ColorSpacePoint colorP = m_pColorCoordinates[i];
				if (colorP.X != -std::numeric_limits<float>::infinity() && colorP.Y != -std::numeric_limits<float>::infinity())
				{
				int colorX = static_cast<int>(colorP.X + 0.5f);
				int colorY = static_cast<int>(colorP.Y + 0.5f);
				if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
				{
				cloud->points[i].b = i_rgb.data[(colorY * 1920 + colorX) * 4];
				cloud->points[i].g = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
				cloud->points[i].r = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
				}
				}*/

				//------д��XYZ------
				CameraSpacePoint cameraP = m_pCameraCoordinates[i];
				if (cameraP.X != -std::numeric_limits<float>::infinity() && cameraP.Y != -std::numeric_limits<float>::infinity() && cameraP.Z != -std::numeric_limits<float>::infinity())
				{
					float cameraX = static_cast<float>(cameraP.X);
					float cameraY = static_cast<float>(cameraP.Y);
					float cameraZ = static_cast<float>(cameraP.Z);

					cloud->points[i].x = cameraX;
					cloud->points[i].y = cameraY;
					cloud->points[i].z = cameraZ;

				}
			}
		}
		//-----------------------��ȡ��Χ�ڵĵ�------------------------
		pcl::ConditionAnd<MyPointDataType>::Ptr range_cond(new pcl::ConditionAnd<MyPointDataType>());
		range_cond->addComparison(pcl::FieldComparison<MyPointDataType>::ConstPtr(new	pcl::FieldComparison<MyPointDataType>("z", pcl::ComparisonOps::GT, 0.001)));
		range_cond->addComparison(pcl::FieldComparison<MyPointDataType>::ConstPtr(new	pcl::FieldComparison<MyPointDataType>("z", pcl::ComparisonOps::LT, 2.0)));
		range_cond->addComparison(pcl::FieldComparison<MyPointDataType>::ConstPtr(new	pcl::FieldComparison<MyPointDataType>("x", pcl::ComparisonOps::GT, -0.5)));
		range_cond->addComparison(pcl::FieldComparison<MyPointDataType>::ConstPtr(new	pcl::FieldComparison<MyPointDataType>("x", pcl::ComparisonOps::LT, 0.5)));
		//range_cond->addComparison(pcl::FieldComparison<MyPointDataType>::ConstPtr(new	pcl::FieldComparison<MyPointDataType>("y", pcl::ComparisonOps::GT, -0.85)));
		pcl::ConditionalRemoval<MyPointDataType> condrem(range_cond, false);
		condrem.setInputCloud(cloud);
		condrem.setKeepOrganized(false);
		condrem.filter(*cloud_filtered);
		//--------------------------------------------------------------

		//-----------------------ȥ����Ⱥ��------------------------
		//pcl::RadiusOutlierRemoval<MyPointDataType> outrem;
		//outrem.setInputCloud(cloud_filtered);
		//outrem.setRadiusSearch(0.03);
		//outrem.setMinNeighborsInRadius(15);
		//outrem.filter(*cloud_filtered);

		//pcl::StatisticalOutlierRemoval<MyPointDataType> sor;
		//sor.setInputCloud(cloud_filtered);
		//sor.setMeanK(10);
		//sor.setStddevMulThresh(1.0);
		//sor.filter(*cloud_filtered);
		//--------------------------------------------------------------

		string s = "cow";
		s += num2str(count);
		s += ".pcd";
		pcl::io::savePCDFile(s, *cloud_filtered, false); //�����Ʊ��浽PCD�ļ���
		std::cerr << "Saved " << cloud_filtered->points.size() << " data points." << std::endl;
		s.clear();

		//Beep(1046, 1000);

		// ��ʾ���ͼ
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
		viewer->addPointCloud(cloud_filtered);
		viewer->resetCamera();

		viewer->addCoordinateSystem(0.1);
		viewer->initCameraParameters();
		while (!viewer->wasStopped()) {
			viewer->spinOnce();
		}

		count++;
		cout << "test" << endl;
		//system("pause");
		// �ͷ���Դ
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pMultiFrame);
	}


	// �رմ��ڣ��豸
	m_pKinectSensor->Close();
	SafeRelease(m_pKinectSensor);
	std::system("pause");
	return 0;
}