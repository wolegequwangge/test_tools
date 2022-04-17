#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>

int main(int argc,char** argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(argv[1], *cloud_input)) {
        std::cerr << "cannot open file pcd" << std::endl;
        return 0;
    }
    // if (pcl::io::loadPCDFile(argv[2], *cloud_transform)) {
    //     std::cerr << "cannot open file pcd" << std::endl;
    //     return 0;
    // }

    // std::size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
	// std::mt19937 rand_num(seed);
    // std::uniform_real_distribution<double> dist_theta(0,20);
    // double theta = dist_theta(rand_num) / 180 * M_PI;
    // std::cout << "theta is " << std::endl << theta << std::endl;
    // Eigen::Vector3d axis = Eigen::Vector3d::Random();

    Eigen::Vector3d axis;
    axis << -0.510976, -0.859595, 0;
    double theta = 0.0276438;
    Eigen::AngleAxisd aa(theta,axis);
    Eigen::Quaterniond q(aa);

    // std::uniform_real_distribution<double> dist_t(0,3);
    // Eigen::Translation3d _t(dist_t(rand_num),dist_t(rand_num),dist_t(rand_num));

    // Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();

    Eigen::Translation3d _t(0,0,0);
    Eigen::Affine3d aff = _t*q.toRotationMatrix();
    // Eigen::Affine3d aff = pcl::getTransformation(0.1, 0.1, 0.1, 0.2, 0.2, 0.2).cast<double>();

    Eigen::Matrix4d _T = aff.matrix();
    std::cout << "Affine3d is " << std::endl << _T << std::endl;

    pcl::transformPointCloud(*cloud_input, *cloud_transform, _T);
    double x_max(std::numeric_limits<double>::min()), y_max(std::numeric_limits<double>::min()), z_max(std::numeric_limits<double>::min());
    double x_min(std::numeric_limits<double>::max()), y_min(std::numeric_limits<double>::max()), z_min(std::numeric_limits<double>::max());

#pragma omp for
    for(size_t i=0; i<cloud_transform->size(); i++) {
        x_max = cloud_transform->points[i].x > x_max?cloud_transform->points[i].x:x_max;
        y_max = cloud_transform->points[i].y > y_max?cloud_transform->points[i].y:y_max;
        z_max = cloud_transform->points[i].z > z_max?cloud_transform->points[i].z:z_max;
        
        x_min = cloud_transform->points[i].x < x_min?cloud_transform->points[i].x:x_min;
        y_min = cloud_transform->points[i].y < y_min?cloud_transform->points[i].y:y_min;
        z_min = cloud_transform->points[i].z < z_min?cloud_transform->points[i].z:z_min;
    }
    std::cout << "max: " << x_max << "," << y_max << "," << z_max << std::endl;
    std::cout << "min: " << x_min << "," << y_min << "," << z_min << std::endl;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_transform);
    std::vector<int> indices;
#pragma omp for
    for(size_t i=0; i<cloud_transform->size(); i++){
        if(cloud_transform->points[i].z > 1.6) indices.push_back(i);
    }
    extract.setIndices(boost::make_shared<std::vector<int>>(indices));
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloud_crop);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("transform"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_ch(cloud_input, 255, 0, 0);
    // viewer->addPointCloud(cloud_input, red_ch, "cloud_in");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_ch(cloud_crop, 0, 255, 0);
    viewer->addPointCloud(cloud_crop, green_ch, "cloud_transform");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_ch(cloud_transform, 0, 255, 0);
    // viewer->addPointCloud(cloud_transform, green_ch, "cloud_transform");
    viewer->addCoordinateSystem();
    // Create two vertically separated viewports
    // int v1(0);
    // int v2(1);
    // viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    // viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // // The color we will be using
    // float bckgr_gray_level = 0.0; // Black
    // float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // // Original point cloud is white
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_input, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
    // viewer->addPointCloud(cloud_input, cloud_in_color_h, "cloud_in_v1", v1);
    // viewer->addPointCloud(cloud_input, cloud_in_color_h, "cloud_in_v2", v2);

    // // Transformed point cloud is green
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_bak, 20, 180, 20);
    // viewer->addPointCloud(cloud_bak, cloud_tr_color_h, "cloud_bak_v1", v1);

    // // ICP aligned point cloud is red
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(cloud_transform, 180, 20, 20);
    // viewer->addPointCloud(cloud_transform, cloud_icp_color_h, "cloud_tr_v2", v2);

    while(!viewer->wasStopped()) viewer->spin();
    return 0;
}   