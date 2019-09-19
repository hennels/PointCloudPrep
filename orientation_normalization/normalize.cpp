#include <iostream>
#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, "normalized");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normalized");
  viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

Eigen::Matrix4f Rmat(double start_x, double start_y, double start_z, double end_x, double end_y, double end_z){
  double start_mag = sqrt(start_x*start_x + start_y*start_y + start_z*start_z);
  double end_mag = sqrt(end_x*end_x + end_y*end_y + end_z*end_z);
  start_x = start_x/start_mag;
  start_y = start_y/start_mag;
  start_z = start_z/start_mag;
  end_x = end_x/end_mag;
  end_y = end_y/end_mag;
  end_z = end_z/end_mag;
  double k_x = start_y*end_z - start_z*end_y;
  double k_y = start_z*end_x - start_x*end_z;
  double k_z = start_x*end_y - start_y*end_x;
  double sin_theta = sqrt(k_x*k_x + k_y*k_y + k_z*k_z);
  k_x = k_x/sin_theta;
  k_y = k_y/sin_theta;
  k_z = k_z/sin_theta;
  double cos_theta = start_x*end_x + start_y*end_y + start_z*end_z;
  double o_m_cos_theta = 1.0-cos_theta;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // sin terms
  transform(0,1) = transform(0,1)-k_z*sin_theta;
  transform(0,2) = transform(0,2)+k_y*sin_theta;
  transform(1,0) = transform(1,0)+k_z*sin_theta;
  transform(1,2) = transform(1,2)-k_x*sin_theta;
  transform(2,0) = transform(2,0)-k_y*sin_theta;
  transform(2,1) = transform(2,1)+k_x*sin_theta;
  // cos terms
  transform(0,0) = transform(0,0)-(k_z*k_z+k_y*k_y)*o_m_cos_theta;
  transform(0,1) = transform(0,1)+k_x*k_y*o_m_cos_theta;
  transform(0,2) = transform(0,2)+k_x*k_z*o_m_cos_theta;
  transform(1,0) = transform(1,0)+k_x*k_y*o_m_cos_theta;
  transform(1,1) = transform(1,1)-(k_z*k_z+k_x*k_x)*o_m_cos_theta;
  transform(1,2) = transform(1,2)+k_y*k_z*o_m_cos_theta;
  transform(2,0) = transform(2,0)+k_x*k_z*o_m_cos_theta;
  transform(2,1) = transform(2,1)+k_y*k_z*o_m_cos_theta;
  transform(2,2) = transform(2,2)-(k_y*k_y+k_x*k_x)*o_m_cos_theta;
  return transform;
}

Eigen::Matrix4f center_scale(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
  Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
  double cloud_size = (double) cloud->points.size();
  if (cloud->points.size() > 0){
    double mu_x, mu_y, mu_z, dist, x, y, z, tmp_dist;
    for (size_t i = 0; i < cloud->points.size(); i++){
      mu_x += (double) cloud->points[i].x;
      mu_y += (double) cloud->points[i].y;
      mu_z += (double) cloud->points[i].z;
    }
    transform1(0,3) = -1*mu_x/cloud_size;
    transform1(1,3) = -1*mu_y/cloud_size;
    transform1(2,3) = -1*mu_z/cloud_size;
    dist = 0.0;
    for (size_t i = 0; i < cloud->points.size(); i++){
      x = ((double) cloud->points[i].x) + transform1(0,3);
      y = ((double) cloud->points[i].y) + transform1(1,3);
      z = ((double) cloud->points[i].z) + transform1(2,3);
      tmp_dist = (x*x + y*y + z*z);
      if (tmp_dist > dist){
        dist = tmp_dist;
      }
    }
    dist = 0.999/sqrt(dist);
    transform2(0,0) = dist;
    transform2(1,1) = dist;
    transform2(2,2) = dist;
  }
  return transform2*transform1;
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
  double flip = 1.0;
  if (pcl::console::find_switch (argc, argv, "-f")){
    flip = -1.0;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

  if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
    std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
    showHelp (argv[0]);
    return -1;
  }

  Eigen::Matrix4f Tcenter_scale = center_scale(cloud);
  //std::cerr << "Transform:\n" << Tcenter_scale << std::endl;
  pcl::transformPointCloud (*cloud, *cloud, Tcenter_scale);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a linear model for the given dataset.");
    return (-1);
  }
  /*
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
				      << coefficients->values[3] << " "
                                      << coefficients->values[4] << " "
                                      << coefficients->values[5] << std::endl;
  */
  Eigen::Matrix4f transform = Rmat((double) coefficients->values[3], (double) coefficients->values[4], (double) coefficients->values[5], 0.0, 0.0, flip);
  //std::cerr << "Transform:\n" << transform << std::endl;
  //pcl::copyPointCloud<pcl::PointXYZRGBA>(*cloud, *inliers, *cloud_filtered);
  pcl::transformPointCloud (*cloud, *cloud, transform);
  //seg.segment (*inliers, *coefficients);
  //if (inliers->indices.size () == 0)
  //{
  //  PCL_ERROR ("Could not estimate a linear model for the given dataset.");
  //  return (-1);
  //}
  /*
  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
				      << coefficients->values[3] << " "
                                      << coefficients->values[4] << " "
                                      << coefficients->values[5] << std::endl;*/
  
  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZRGBA> (argv[filenames[1]], *cloud, false);
  
  //pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud_filtered);
  /*
  while (!viewer->wasStopped ()){
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100);
  }*/
  pcl::visualization::PCLVisualizer::Ptr viewer = simpleVis(cloud);
  while (!viewer->wasStopped ()){
    viewer->spinOnce (100);
    //std::this_thread::sleep_for(100);
  }
  return (0);
}

