#include <iostream>
#include <random>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/registration/icp.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_block_generate(int _s, double _a, double _b, double _sigma, int _i, double _shift_x=0, double _shift_y=0){
    std::size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::mt19937 rand_num(seed);
	
    std::uniform_real_distribution<double> dist_plane(_a,_b);
    std::normal_distribution<double> dist_z(0, _sigma);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_output->width = _s;
    cloud_output->height = 1;
    cloud_output->resize(cloud_output->width * cloud_output->height);
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = cloud_output->points.begin(); pt < cloud_output->points.end(); pt++) {
        pt->z = dist_plane(rand_num) + _shift_x;
        pt->y = dist_plane(rand_num) + _shift_y;
        pt->x = dist_z(rand_num);
        pt->intensity = _i;
    }
    return cloud_output;
}

int main(int argc,char** argv){

    // std::size_t seed_x = std::chrono::system_clock::now().time_since_epoch().count();
	// std::mt19937 rand_num_x(seed_x);
    // std::size_t seed_y = std::chrono::system_clock::now().time_since_epoch().count();
	// std::mt19937 rand_num_y(seed_y);
    // std::size_t seed_z = std::chrono::system_clock::now().time_since_epoch().count();
	// std::mt19937 rand_num_z(seed_z);
    
    // std::size_t seed = std::chrono::system_clock::now().time_since_epoch().count();
	// std::mt19937 rand_num(seed);
	
    // std::uniform_real_distribution<double> dist_plane(-0.1,0.1); // -1到1均匀分布
    // std::normal_distribution<double> dist_z(0, 10e-4);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_0(new pcl::PointCloud<pcl::PointXYZI>);
    // cloud_generate_0->width = 22500;
    // cloud_generate_0->height = 1;
    // cloud_generate_0->resize(cloud_generate_0->width * cloud_generate_0->height);
    // for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = cloud_generate_0->points.begin(); pt < cloud_generate_0->points.end(); pt++) {
    //     pt->x = dist_plane(rand_num);
    //     pt->y = dist_plane(rand_num);
    //     pt->z = dist_z(rand_num);
    //     pt->intensity = 20;
    // }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_1(new pcl::PointCloud<pcl::PointXYZI>);
    // cloud_generate_1->width = 22500;
    // cloud_generate_1->height = 1;
    // cloud_generate_1->resize(cloud_generate_1->width * cloud_generate_1->height);
    // for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = cloud_generate_1->points.begin(); pt < cloud_generate_1->points.end(); pt++) {
    //     pt->x = dist_plane(rand_num)+0.2;
    //     pt->y = dist_plane(rand_num)+0.2;
    //     pt->z = dist_z(rand_num);
    //     pt->intensity = 100;
    // }

    // pcl::PointCloud<pcl::PointXYZI> cloud_generate_0;
    // cloud_generate_0.width = 2250;
    // cloud_generate_0.height = 1;
    // std::cout << cloud_generate_0.size() << std::endl;
    // for (std::size_t i(0);i<cloud_generate_0.size();i++) {
    //     cloud_generate_0[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud_generate_0[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //     cloud_generate_0[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    // }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_00(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_02(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_11(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_13(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_14(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_21(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_22(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_30(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_32(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_34(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_40(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_41(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_42(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_generate_final(new pcl::PointCloud<pcl::PointXYZI>);

    double square_length(0.07);
    double lower(-1*square_length);
    double upper(square_length);
    
    cloud_generate_00 = cloud_block_generate(2250, lower, upper, 20e-4, 150, -4*square_length, 4*square_length);
    cloud_generate_02 = cloud_block_generate(2250, lower, upper, 20e-4, 150, 0, 4*square_length);
    cloud_generate_11 = cloud_block_generate(2250, lower, upper, 20e-4, 150, -2*square_length, 2*square_length);
    cloud_generate_13 = cloud_block_generate(2250, lower, upper, 20e-4, 150, 2*square_length, 2*square_length);
    cloud_generate_14 = cloud_block_generate(2250, lower, upper, 20e-4, 150, 4*square_length, 2*square_length);
    cloud_generate_21 = cloud_block_generate(2250, lower, upper, 20e-4, 150, -2*square_length);
    cloud_generate_22 = cloud_block_generate(2250, lower, upper, 20e-4, 150);
    cloud_generate_30 = cloud_block_generate(2250, lower, upper, 20e-4, 150, -4*square_length,-2*square_length);
    cloud_generate_32 = cloud_block_generate(2250, lower, upper, 20e-4, 150, 0, -2*square_length);
    cloud_generate_34 = cloud_block_generate(2250, lower, upper, 20e-4, 150, 4*square_length, -2*square_length);
    cloud_generate_40 = cloud_block_generate(2250, lower, upper, 20e-4, 150, -4*square_length, -4*square_length);
    cloud_generate_41 = cloud_block_generate(2250, lower, upper, 20e-4, 150, -2*square_length, -4*square_length);
    cloud_generate_42 = cloud_block_generate(2250, lower, upper, 20e-4, 150, 0, -4*square_length);

    *cloud_generate_final += *cloud_generate_00;
    *cloud_generate_final += *cloud_generate_02;
    *cloud_generate_final += *cloud_generate_11;
    *cloud_generate_final += *cloud_generate_13;
    *cloud_generate_final += *cloud_generate_14;
    *cloud_generate_final += *cloud_generate_21;
    *cloud_generate_final += *cloud_generate_22;
    *cloud_generate_final += *cloud_generate_30;
    *cloud_generate_final += *cloud_generate_32;
    *cloud_generate_final += *cloud_generate_34;
    *cloud_generate_final += *cloud_generate_40;
    *cloud_generate_final += *cloud_generate_41;
    *cloud_generate_final += *cloud_generate_42;

    Eigen::Affine3d _T = pcl::getTransformation(0,0,0,M_PI/2,0,0).cast<double>();
    pcl::transformPointCloud(*cloud_generate_final, *cloud_generate_final, _T);
    pcl::io::savePCDFileASCII("../generated.pcd", *cloud_generate_final);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar(new pcl::PointCloud<pcl::PointXYZI>);
    // if (pcl::io::loadPCDFile(argv[1], *cloud_lidar)) {
    //     std::cerr << "cannot open file pcd" << std::endl;
    //     return 0;
    // }
    // *cloud_generate_final += *cloud_lidar;
    pcl::visualization::CloudViewer generate_viewer("generated cloud");
    while (!generate_viewer.wasStopped())
        generate_viewer.showCloud(cloud_generate_final);
    
    return 0;
}