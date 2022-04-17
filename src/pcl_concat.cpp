#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

void findMaxIntensity(float& max_intensity, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in) {
#pragma omp for
    for(size_t i = 0; i < cloud_in->size(); i++)
        max_intensity = cloud_in->points[i].intensity > max_intensity ? cloud_in->points[i].intensity : max_intensity;
    std::cout << "max_intensity is " << max_intensity << std::endl;
    return;
}

void intensity2rgb(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, float max_intensity, float s, float v) {
#pragma omp for
    for(size_t i = 0; i < cloud_in->size(); i++){
        pcl::PointXYZHSV p;
        p.x = cloud_in->points[i].x;
        p.y = cloud_in->points[i].y;
        p.z = cloud_in->points[i].z;
        p.s = s;
        p.v = v;
        p.h = 360 * cloud_in->points[i].intensity / max_intensity;
        pcl::PointXYZRGB p_rgb;
        pcl::PointXYZHSVtoXYZRGB(p, p_rgb);
        cloud_out->points.push_back(p_rgb);
    }
    std::cout << "cloud_concat->size() is " << cloud_out->size() << std::endl;
}

int main(int argc, char** argv) {
    
    if (argc != 4) {
        std::cerr << "please enter 'pcd path' !" << std::endl;
        return 0;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_concat(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(argv[1], *cloud1);
    std::cout << "cloud1->size() is " << cloud1->size() << std::endl;

    float thetaz = M_PI * 95 / 180;
    float thetax = M_PI * 181 / 180;
    Eigen::Quaternionf q1 = Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX());
    Eigen::Translation3f _t(1.65,-0.05,2);
    Eigen::Matrix4f _T = (_t * q1.toRotationMatrix()).matrix();
    std::cout << "Affine3f is " << std::endl << _T << std::endl;
    pcl::transformPointCloud(*cloud1, *cloud1, _T);

    float max_intensity1 = 0.f;
    findMaxIntensity(max_intensity1, cloud1);
    intensity2rgb(cloud1, cloud_concat, max_intensity1, 0.2, 0.8);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(argv[2], *cloud2);
    std::cout << "cloud2->size() is " << cloud2->size() << std::endl;
    float max_intensity2 = 0.f;
    findMaxIntensity(max_intensity2, cloud2);
    intensity2rgb(cloud2, cloud_concat, max_intensity2, 0.8, 0.5);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile(argv[3], *cloud3);
    std::cout << "cloud3->size() is " << cloud3->size() << std::endl;

    *cloud_concat += *cloud3;
    pcl::io::savePCDFileBinary("res.pcd", *cloud_concat);
    pcl::visualization::CloudViewer concat_viewer("cloud_concat");
    while(!concat_viewer.wasStopped()) concat_viewer.showCloud(cloud_concat);
    return 0;
}