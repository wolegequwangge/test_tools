#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// typedef pcl::PointXYZI PointT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "please enter pcd path!" << std::endl;
        return -1;
    }
    CloudT::Ptr cloud_in(new CloudT());
    pcl::io::loadPCDFile(argv[1], *cloud_in);
    // pcl::io::loadPLYFile(argv[1], *cloud_in);s
    size_t cloud_size = cloud_in->size();
    std::cout << "cloud_in's size is: " << cloud_size << std::endl;

    double x_arg(0),y_arg(0),z_arg(0);
#pragma omp for
    for(size_t i = 0; i < cloud_size; i++) {
        x_arg+=cloud_in->points[i].x;
        y_arg+=cloud_in->points[i].y;
        z_arg+=cloud_in->points[i].z;
    }
    x_arg/=cloud_size;
    y_arg/=cloud_size;
    z_arg/=cloud_size;
    std::cout << "(x_arg,y_arg,z_arg): (" << x_arg << y_arg << z_arg << ")" << std::endl;

    Eigen::Matrix3Xd A_nor;
    A_nor.resize(3, cloud_size);
#pragma omp for
    for(size_t i = 0; i < cloud_size; i++) {
        A_nor(0,i) = cloud_in->points[i].x - x_arg;
        A_nor(1,i) = cloud_in->points[i].y - y_arg;
        A_nor(2,i) = cloud_in->points[i].z - z_arg;
    }
    std::cout << "A_nor's size is " << A_nor.size() << std::endl;
    
    Eigen::BDCSVD<Eigen::Matrix3Xd> svd(A_nor, Eigen::ComputeThinU);
    std::cout << "A_nor's SVD sigma is: " << svd.singularValues().transpose() << std::endl;
    std::cout << "A_nor's SVD u1 is: " << svd.matrixU().col(0).transpose() << std::endl;
    std::cout << "A_nor's SVD u2 is: " << svd.matrixU().col(1).transpose() << std::endl;
    std::cout << "A_nor's SVD u3 is: " << svd.matrixU().col(2).transpose() << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(new pcl::visualization::PCLVisualizer("pca"));
    
    PointT origin(0,0,0);
    PointT u1(svd.matrixU().col(0)[0],svd.matrixU().col(0)[1],svd.matrixU().col(0)[2]);
    PointT u2(svd.matrixU().col(1)[0],svd.matrixU().col(1)[1],svd.matrixU().col(1)[2]);
    PointT u3(svd.matrixU().col(2)[0],svd.matrixU().col(2)[1],svd.matrixU().col(2)[2]);

    viewer_ptr->addPointCloud(cloud_in);
    std::string su1("u1"), su2("u2"), su3("u3");
    viewer_ptr->addArrow(origin, u1, 255, 0, 0, su1);
    viewer_ptr->addArrow(origin, u2, 0, 255, 0, su2);
    viewer_ptr->addArrow(origin, u3, 0, 0, 255, su3);

    while(!viewer_ptr->wasStopped()) viewer_ptr->spinOnce();
    return 0;
}
