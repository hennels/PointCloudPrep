#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
  std::cout << "Usage: " << program_name << " in_filename.ply out_filename.ply" << std::endl;
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

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (15);
  sor.setStddevMulThresh (3.5);
  sor.filter (*cloud_filtered);

  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZRGBA> (argv[filenames[1]], *cloud_filtered, false);

  return (0);
}

