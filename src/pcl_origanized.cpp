#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv){
    if (argc != 2) {
        std::cerr << "please enter img path!" << std::endl;
        return 0;
    }

    cv::Mat src_image = cv::imread(argv[1]);
    std::cout << "image type: " << src_image.type() << std::endl;
    int width = src_image.cols;
    int height = src_image.rows;
    std::cout << "image width: " << width << std::endl;
    std::cout << "image height: " << height << std::endl;

    // cv::imshow("src", src_image);
    // while(1) if (cv::waitKey(1) == 27) break;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr origan_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // origan_cloud->points.resize(src_image.rows * src_image.cols);
    origan_cloud->resize(width * height);
    std::cout << "cloud size: " << origan_cloud->size() << std::endl;
    origan_cloud->width = width;
    origan_cloud->height = height;
    std::cout << "cloud width: " << origan_cloud->width << std::endl;
    std::cout << "cloud height: " << origan_cloud->height << std::endl;

#pragma omp for
    for(size_t i = 0; i < origan_cloud->size(); i++){
        int u = i / width;
        int v = i % width;
        origan_cloud->points[i].x = u - 200;
        origan_cloud->points[i].y = v - 300;
        origan_cloud->points[i].z = 0;

        // origan_cloud->points[i].b = src_image.at<cv::Vec3w>(u, v)[0];
        // origan_cloud->points[i].g = src_image.at<cv::Vec3w>(u, v)[1];
        // origan_cloud->points[i].r = src_image.at<cv::Vec3w>(u, v)[2];        

        origan_cloud->points[i].b = src_image.ptr<uchar>(u)[v*3];
        origan_cloud->points[i].g = src_image.ptr<uchar>(u)[v*3+1];
        origan_cloud->points[i].r = src_image.ptr<uchar>(u)[v*3+2];
    }

    pcl::visualization::CloudViewer origanized_viewer("origanized cloud");
    while (!origanized_viewer.wasStopped()) 
        origanized_viewer.showCloud(origan_cloud);

    return 0;
}