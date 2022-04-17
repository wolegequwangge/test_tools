#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/bilateral.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "please enter 'pcd path' !" << std::endl;
        return 0;
    }
    PointCloudT::Ptr cloud_raw(new PointCloudT());
    pcl::io::loadPCDFile(argv[1], *cloud_raw);

    pcl::visualization::CloudViewer raw_viewer("raw_cloud");
    while (!raw_viewer.wasStopped()) raw_viewer.showCloud(cloud_raw);  
    
    // PointCloudT::Ptr cloud_sampler(new PointCloudT());
    // pcl::VoxelGrid<pcl::PointXYZI> voxel_sampler;
    // voxel_sampler.setInputCloud(cloud_raw);
    // voxel_sampler.setLeafSize(0.05f, 0.05f, 0.05f);
    // voxel_sampler.filter(*cloud_sampler);
    // pcl::visualization::CloudViewer sampler_viewer("sampler_cloud");
    // while (!sampler_viewer.wasStopped()) sampler_viewer.showCloud(cloud_sampler);
    
    // PointCloudT::Ptr cloud_sor(new PointCloudT());
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    //     sor.setInputCloud (cloud_sampler);
    //     sor.setMeanK (250);
    //     sor.setStddevMulThresh (0.5);
    //     sor.filter (*cloud_sor);
    // pcl::visualization::CloudViewer sor_viewer("sor_cloud");
    // while (!sor_viewer.wasStopped()) sor_viewer.showCloud(cloud_sor);

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud_raw);

    PointCloudT::Ptr cloud_bil(new PointCloudT());
    std::cout << "bilateral filter start" << std::endl;
    pcl::BilateralFilter<PointT> bil;
    bil.setInputCloud(cloud_raw);
    bil.setStdDev(std::stof(argv[2]));
    bil.setHalfSize(std::stof(argv[3]));
    bil.setSearchMethod(tree);
    bil.filter(*cloud_bil);
    pcl::visualization::CloudViewer bil_viewer("bil_cloud");
    while (!bil_viewer.wasStopped()) bil_viewer.showCloud(cloud_bil);

    return 0;
}