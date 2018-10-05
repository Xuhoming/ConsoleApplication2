#include <iostream> //��׼����/���
#include <pcl/io/pcd_io.h> //pcd�ļ�����/���
#include <pcl/point_types.h> //���ֵ�����
#include <pcl/registration/icp.h> //ICP(iterative closest point)��׼

int main(int argc, char** argv)
{
	//��������ָ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //����������ƣ�ָ�룩
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>); //�������/Ŀ����ƣ�ָ�룩
	pcl::io::loadPCDFile("cow0_withNormal.pcd", *cloud_in);
	pcl::io::loadPCDFile("cow1_withNormal.pcd", *cloud_out);																   //���ɲ�������cloud_in
	//*********************************
	// ICP��׼
	//*********************************
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //����ICP��������ICP��׼
	icp.setInputCloud(cloud_in); //�����������
	icp.setInputTarget(cloud_out); //����Ŀ����ƣ�������ƽ��з���任���õ�Ŀ����ƣ�
	pcl::PointCloud<pcl::PointXYZ> Final; //�洢���
										  //������׼������洢��Final��
	icp.align(Final);
	//���ICP��׼����Ϣ���Ƿ���������϶ȣ�
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	//������յı任����4x4��
	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}