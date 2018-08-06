#include "compressionShowHelper.h"

int main(int argc, char** argv)
{
	std::cout << "Read and display the point cloud data file" << std::endl;

	if (argc < 3)
	{
		std::cout << "Command USAGE:" << std::endl << "ShowPointClouds <original file name> <decompressed file name>" << std::endl;
		return 1;
	}

	compressionShowHelper originalFile(argv[1], "original file", new pcl::PointCloud<pcl::PointXYZ>);
	compressionShowHelper decompressedFile(argv[2], "decompression file", new pcl::PointCloud<pcl::PointXYZ>);

	originalFile.init();
	decompressedFile.init();
	//decompressedFile.coordTransform();//coordinate has changed after the decompression, normalize it to [0, 1].

	originalFile.coordToFile();
	decompressedFile.coordToFile();

	originalFile.printCloudInfo();
	decompressedFile.printCloudInfo();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = compressionShowHelper::doubleViews(originalFile.getPointcloud(), decompressedFile.getPointcloud());

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(10000));
	}

	return (0);
}