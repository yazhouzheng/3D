#include "compressionShowHelper.h"

compressionShowHelper::compressionShowHelper(const std::string& inputFileName, const std::string& outputFileName, pcl::PointCloud<pcl::PointXYZ>* pointcloud)
	:inputFileName(inputFileName), outputFileName(outputFileName), pointcloud(pointcloud)
{
}

compressionShowHelper::~compressionShowHelper()
{
}

bool compressionShowHelper::init()
{
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(inputFileName, *pointcloud) == -1)
	{
		PCL_ERROR("Couldn't read original file %s \n", inputFileName);
		return (-1);
	}
}

void compressionShowHelper::coordTransform()
{
	for (size_t i = 0; i < pointcloud->points.size(); ++i)
	{
		pointcloud->points[i].x *= 0.001;
		pointcloud->points[i].y *= 0.001;
		pointcloud->points[i].z *= 0.001;
	}
}

void compressionShowHelper::printCloudInfo()
{
	std::cout << "There are " << pointcloud->width * pointcloud->height
		<< " data points " << inputFileName << " in file." << std::endl;
}

bool compressionShowHelper::coordToFile() 
{
		std::ofstream out_file(outputFileName, std::ios::binary);
		if (!out_file) 
		{
			printf("Failed to create the output file.\n");
			return -1;
		}

		//Output all the point cloud to a file of original ply file.
		for (size_t i = 0; i < pointcloud->points.size(); ++i)
			out_file << " " << pointcloud->points[i].x
			<< " " << pointcloud->points[i].y
			<< " " << pointcloud->points[i].z << std::endl;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> compressionShowHelper::singleView()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(pointcloud, "cloud");
	return (viewer);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr compressionShowHelper::getPointcloud()
{
	return pointcloud;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> compressionShowHelper::doubleViews(
	pointcloudPtr originalCloud, pointcloudPtr decompressedCloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	std::string view1text = "point cloud number:" + std::to_string(originalCloud->width * originalCloud->height);
	viewer->addText(view1text, 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(originalCloud, 238, 233, 191);
	viewer->addPointCloud<pcl::PointXYZ>(originalCloud, source_color, "originalCloud", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.2, v2);
	std::string view2text = "point cloud number:" + std::to_string(decompressedCloud->width * decompressedCloud->height);
	viewer->addText(view2text, 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(decompressedCloud, 99, 184, 255);
	viewer->addPointCloud<pcl::PointXYZ>(decompressedCloud, target_color, "decompressedCloud", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "originalCloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "decompressedCloud");
	//viewer->addCoordinateSystem(1.0);

	//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	//viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}