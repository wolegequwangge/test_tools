#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr deleted_area(new pcl::PointCloud<pcl::PointXYZI>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> origin_viewer(new pcl::visualization::PCLVisualizer("origin_viewer"));
pcl::ExtractIndices<pcl::PointXYZI> extract;
std::vector< int > indices;
// int k = 0; 
void area_pick_callback(const pcl::visualization::AreaPickingEvent& event, void* args){
    // std::vector< int > indices;
    std::vector<int> new_indices;
    if (event.getPointsIndices(new_indices)==-1)
        return;
    // Extract the inliers
    
    indices.insert(indices.end(),new_indices.begin(),new_indices.end());
    //* indices默认已排序
    // std::sort(indices.begin(),indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices);
    extract.setIndices (index_ptr);
    extract.setNegative (true);//如果设为true,可以提取指定index之外的点云
    extract.filter (*deleted_area);

    deleted_area->width = 1;
    deleted_area->height = deleted_area->points.size();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> white_ch(deleted_area, 255, 255, 255);
 
    origin_viewer->removePointCloud("deleted");
    origin_viewer->addPointCloud(deleted_area, white_ch, "deleted");
}
 
int main(int argc,char** argv){
    if (pcl::io::loadPCDFile(argv[1], *cloud_origin)) {
        std::cerr << "cannot open file origin.pcd" << std::endl;
        return 0;
    }
    extract.setInputCloud (cloud_origin);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud_origin,"intensity");
    origin_viewer->addPointCloud(cloud_origin, intensity_color, "origin");
    origin_viewer->registerAreaPickingCallback(area_pick_callback, (void*)&cloud_origin);
 
    while (!origin_viewer->wasStopped())
        origin_viewer->spin();
    pcl::io::savePCDFileASCII("../deleted.pcd", *deleted_area);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_deleted(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile("../deleted.pcd", *cloud_deleted)) {
        std::cerr << "cannot open file deleted.pcd" << std::endl;
        return 0;
    }
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> _intensity_color(cloud_deleted,"intensity");
    pcl::visualization::PCLVisualizer deleted_viewer("deleted_viewer");
    deleted_viewer.addPointCloud(cloud_deleted, _intensity_color, "deleted");
    
    while (!deleted_viewer.wasStopped())
        deleted_viewer.spin();

    return 0;
}