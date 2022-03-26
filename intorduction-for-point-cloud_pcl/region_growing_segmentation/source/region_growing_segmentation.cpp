#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <windows.h>
#include <stdio.h>
#include <psapi.h>
void PrintMemoryInfo()
{
	HANDLE hProcess;
	PROCESS_MEMORY_COUNTERS pmc;

	hProcess = GetCurrentProcess();
	printf("\nProcess ID: %u\n", hProcess);

	// Print information about the memory usage of the process.
	//�������ʹ�õ��ڴ���Ϣ

	if (NULL == hProcess)
		return;

	if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc)))
	{
		printf("\tPageFaultCount: 0x%08X\n", pmc.PageFaultCount);
		printf("\tPeakWorkingSetSize: 0x%08X\n",
			pmc.PeakWorkingSetSize);
		printf("\tWorkingSetSize: 0x%08X\n", pmc.WorkingSetSize);
		printf("\tQuotaPeakPagedPoolUsage: 0x%08X\n",
			pmc.QuotaPeakPagedPoolUsage);
		printf("\tQuotaPagedPoolUsage: 0x%08X\n",
			pmc.QuotaPagedPoolUsage);
		printf("\tQuotaPeakNonPagedPoolUsage: 0x%08X\n",
			pmc.QuotaPeakNonPagedPoolUsage);
		printf("\tQuotaNonPagedPoolUsage: 0x%08X\n",
			pmc.QuotaNonPagedPoolUsage);
		printf("\tPagefileUsage: 0x%08X\n", pmc.PagefileUsage);
		printf("\tPeakPagefileUsage: 0x%08X\n",
			pmc.PeakPagefileUsage);
	}

	CloseHandle(hProcess);
}

using namespace pcl::console;
int main(int argc, char** argv)
{

	if (argc < 2)
	{
		std::cout << ".exe xx.pcd -kn 50 -bc 0 -fc 10.0 -nc 0 -st 30 -ct 0.05" << endl;
		return 0;
	}//����������С��1���������ʾ
	time_t start, end, diff[5], option;
	start = time(0);
	int KN_normal = 50; //����Ĭ���������
	bool Bool_Cuting = false;//����Ĭ���������

	float far_cuting = 10, near_cuting = 0, SmoothnessThreshold = 30.0, CurvatureThreshold = 0.05;//����Ĭ���������
	parse_argument(argc, argv, "-kn", KN_normal);//�������ڷ��������Ƶ�k������Ŀ
	parse_argument(argc, argv, "-bc", Bool_Cuting);//�Ƿ���Ҫֱͨ�˲�

	parse_argument(argc, argv, "-fc", far_cuting);//����ָ��γ�ȹ��˵ķ�Χ
	parse_argument(argc, argv, "-nc", near_cuting);//����ָ��γ�ȹ��˵ķ�Χ

	parse_argument(argc, argv, "-st", SmoothnessThreshold);//����ƽ����ֵ
	parse_argument(argc, argv, "-ct", CurvatureThreshold);//����������ֵ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile <pcl::PointXYZ>(argv[1], *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return (-1);
	}// ���������������

	end = time(0);
	diff[0] = difftime(end, start);
	PCL_INFO("\Loading pcd file takes(seconds): %d\n", diff[0]);
	//Noraml estimation step(1 parameter)
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);//����һ��ָ��kd����������Ĺ���ָ��
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;//�������߹��ƶ���
	normal_estimator.setSearchMethod(tree);//������������
	normal_estimator.setInputCloud(cloud);//���÷��߹��ƶ�������㼯
	normal_estimator.setKSearch(KN_normal);// �������ڷ��������Ƶ�k������Ŀ
	normal_estimator.compute(*normals);//���㲢���������
	end = time(0);
	diff[1] = difftime(end, start) - diff[0];
	PCL_INFO("\Estimating normal takes(seconds): %d\n", diff[1]);
	//optional step: cutting the part are far from the orignal point in Z direction.2 parameters
	pcl::IndicesPtr indices(new std::vector <int>);//����һ������

	if (Bool_Cuting)//�ж��Ƿ���Ҫֱͨ�˲�
	{

		pcl::PassThrough<pcl::PointXYZ> pass;//����ֱͨ�˲�������
		pass.setInputCloud(cloud);//�����������
		pass.setFilterFieldName("z");//����ָ�����˵�ά��
		pass.setFilterLimits(near_cuting, far_cuting);//����ָ��γ�ȹ��˵ķ�Χ
		pass.filter(*indices);//ִ���˲��������˲����������������
	}


	// ���������㷨��5������
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;//�������������ָ����
	reg.setMinClusterSize(50);//����һ��������Ҫ����С����
	reg.setMaxClusterSize(1000000);//����һ��������Ҫ��������
	reg.setSearchMethod(tree);//������������
	reg.setNumberOfNeighbours(30);//�����������ٽ�����Ŀ
	reg.setInputCloud(cloud);//�����������
	if (Bool_Cuting)reg.setIndices(indices);//ͨ������������ã�ȷ���Ƿ������������
	reg.setInputNormals(normals);//����������Ƶķ�����
	reg.setSmoothnessThreshold(SmoothnessThreshold / 180.0 * M_PI);//����ƽ����ֵ
	reg.setCurvatureThreshold(CurvatureThreshold);//����������ֵ

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);//��ȡ����Ľ�����ָ��������ڵ��������������С�
	end = time(0);
	diff[2] = difftime(end, start) - diff[0] - diff[1];
	PCL_INFO("\Region growing takes(seconds): %d\n", diff[2]);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;//������������
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;//�����һ�����������
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;

	PrintMemoryInfo();
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("���������ָ��");
	viewer.showCloud(colored_cloud);

	std::string filename("C:\\Users\\MIRACLE\\Desktop\\code\\test.pcd");
	pcl::PCDWriter writer;
	writer.write(filename, *colored_cloud);

	while (!viewer.wasStopped())
	{
	}//���п��ӻ�

	return (0);
}
