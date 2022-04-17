#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

#define PI 3.14159265
// typedef pcl::Normal NormalT;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZINormal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
PointCloudT::Ptr cloud_input(new PointCloudT);
NormalCloudT::Ptr cloud_normal (new NormalCloudT);
NormalCloudT::Ptr normal_selected (new NormalCloudT);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(new pcl::visualization::PCLVisualizer("cloud"));

int main(int argc, char** argv){

    // PointCloudT::Ptr cloud_input(new PointCloudT);
    pcl::io::loadPCDFile(argv[1], *cloud_input);

    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setNumberOfThreads(16);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (2);
    ne.setInputCloud (cloud_input);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    // NormalCloudT::Ptr cloud_normal (new NormalCloudT);
    ne.compute (*cloud_normal);
    double nx(0),ny(0),nz(0);
#pragma omp for
    for(size_t i=0; i<cloud_normal->size(); i++) {
        nx+=cloud_normal->points[i].normal_x;
        ny+=cloud_normal->points[i].normal_y;
        nz+=cloud_normal->points[i].normal_z;
    } 
    nx/=cloud_normal->size();
    ny/=cloud_normal->size();
    nz/=cloud_normal->size();
    double norm = nx*nx+ny*ny+nz*nz;
    double theta = acos(nz/norm);
    Eigen::Vector3d axis;
    axis << ny, -nx, 0;
    axis.normalize();
    std::cout << "(nx,ny,nz): (" << nx << "," << ny << "," << nz << ")" << std::endl;
    std::cout << "norm: " << norm << std::endl;
    std::cout << "theta: " << theta << std::endl;
    std::cout << "axis: " << axis.transpose() << std::endl; 

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_color(cloud_input,"intensity");
    viewer_ptr->addPointCloud(cloud_input, intensity_color, "origin");
    viewer_ptr->addPointCloudNormals<PointT, NormalT>(cloud_input, cloud_normal, 1, 0.1, "cloud_normal");
    viewer_ptr->addCoordinateSystem();

    while (!viewer_ptr->wasStopped()) viewer_ptr->spin();
    return 0;

}