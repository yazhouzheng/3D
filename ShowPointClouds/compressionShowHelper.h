#pragma once
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

class compressionShowHelper {

public:
	typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloudPtr;

	compressionShowHelper(const std::string& inputFileName, const std::string& outputFileName, pcl::PointCloud<pcl::PointXYZ>* pointcloud);
	~compressionShowHelper();

	bool init();
	void coordTransform();
	void printCloudInfo();
	bool coordToFile();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> singleView();

	pcl::PointCloud<pcl::PointXYZ>::Ptr getPointcloud();

	static boost::shared_ptr<pcl::visualization::PCLVisualizer> compressionShowHelper::doubleViews(
		pointcloudPtr originalCloud, pointcloudPtr decompressedCloud);

private:
	std::string inputFileName;
	std::string outputFileName;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud;
};