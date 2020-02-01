#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

Eigen::Matrix4f meanZat0(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  if (cloud->points.size() > 0){
    double sum_z = 0;
    for (size_t i = 0; i < cloud->points.size(); i++){
      sum_z += (double) cloud->points[i].z;
    }
    transform(2,3) = -1.0*sum_z/((double) cloud->points.size());
  }
  return transform;
}


/*
  Adapted from Removing sparse outliers using StatisticalOutlierRemoval
          By Radu B. Rusu
  Adapted by Henry Nelson
*/

// This function displays the help
void
showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << "in_filename.ply out_filename.ply" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

int
main (int argc, char** argv)
{
  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PLY files
  std::vector<int> filenames;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

  if (filenames.size() != 2){
    showHelp (argv[0]);
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
    std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
    showHelp (argv[0]);
    return -1;
  }

  pcl::transformPointCloud (*cloud, *cloud, meanZat0(cloud));

  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZRGBA> (argv[filenames[1]], *cloud, false);
  return (0);
}

