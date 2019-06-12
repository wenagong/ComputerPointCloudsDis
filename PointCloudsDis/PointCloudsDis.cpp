// PointCloudsDis.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include <iostream>
#include<pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include<fstream>

using namespace std;

double computerCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!std::isfinite((*cloud)[i].x))
		{
			continue;
		}
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

int main()
{
	ifstream data;
	data.open("PointCloud3D01.TXT",ios::in);

	int rows = -1;
	char buf[200];
	while (!data.eof()) {
		data.getline(buf, sizeof(buf));
		rows++;
	}
	data.clear();
	data.seekg(0, ios::beg);
	//pcl::PointCloud<pcl::PointXYZ>cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = rows;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(cloud->width*cloud->height);
	
	for (int i = 0; i < rows; i++) {
		double num[3];
		data >> num[0];
		data >> num[1];
		data >> num[2];

		cloud->points[i].x = num[0];
		cloud->points[i].y = num[1];
		cloud->points[i].z = num[2];
	}

	double res=computerCloudResolution(cloud);
	cout << "distance=" <<res<<endl<<"rows="<<rows;
}

