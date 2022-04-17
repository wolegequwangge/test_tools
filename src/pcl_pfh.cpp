#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>

#include <pcl/features/linear_least_squares_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// typedef pcl::PointXYZINormal MyNormalType;
typedef pcl::Normal MyNormalType;

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "please enter pcd path!" << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud_input);

    pcl::PointCloud<MyNormalType>::Ptr cloud_normals(new pcl::PointCloud<MyNormalType> ());

    pcl::NormalEstimationOMP<pcl::PointXYZI, MyNormalType> ne;
    ne.setNumberOfThreads(12);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.setInputCloud(cloud_input);
    ne.compute(*cloud_normals);
    std::cout << "compute normals over" << std::endl;
    // pcl::visualization::PCLVisualizer normal_viewer("normal");
    // normal_viewer.addPointCloudNormals<pcl::PointXYZI, MyNormalType>(cloud_input, cloud_normals);
    // while (!normal_viewer.wasStopped()) normal_viewer.spin();

    for(pcl::PointCloud<MyNormalType>::iterator pt = cloud_normals->begin(); pt!=cloud_normals->end(); pt++){
        
    }

    pcl::FPFHEstimation<pcl::PointXYZI, MyNormalType, pcl::FPFHSignature33> pfh;
    pfh.setInputCloud(cloud_input);
    pfh.setInputNormals(cloud_normals);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr _tree(new pcl::search::KdTree<pcl::PointXYZI> ());
    pfh.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfhs(new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    pfh.setRadiusSearch(1);

    // Compute the features
    pfh.compute(*pfhs);

    std::cout << pfhs->size() << std::endl;

    return 0;
}