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
pcl::PointCloud<pcl::PointXYZI>::Ptr selected_area(new pcl::PointCloud<pcl::PointXYZI>);
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
    
    // pcl::PointCloud<pcl::PointXYZI>::Ptr _selected_area(new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices);
    extract.setIndices (index_ptr);
    extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
    extract.filter (*selected_area);

    // for (int i = 0; i < indices.size(); ++i)
    //     selected_area->points.push_back(cloud_origin->points.at(indices[i]));
    // *selected_area = *_selected_area;
    selected_area->width = 1;
    selected_area->height = selected_area->points.size();
    // pcl::io::savePCDFileASCII("../selected.pcd", *selected_area);
    // pcl::io::savePLYFileASCII("../selected.ply", *selected_area);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> white_ch(selected_area, 255, 255, 255);
 
    // std::string selected_times = std::to_string(k++); // 选中点云名称为selected_times，多次框选会增加
    // origin_viewer->addPointCloud(selected_area, white_ch, selected_times);
    
    origin_viewer->removePointCloud("selected");
    origin_viewer->addPointCloud(selected_area, white_ch, "selected");
}
 
int main(int argc,char** argv){
    // string file_name = argv[1];
    if (pcl::io::loadPCDFile(argv[1], *cloud_origin)) {
        std::cerr << "cannot open file origin.pcd" << std::endl;
        return 0;
    }
    // if (pcl::io::loadPLYFile("../cloudnorm_00000.ply", *cloud_origin)) {
    //     std::cerr << "cannot open file origin.ply" << std::endl;
    //     return 0;
    // }
    extract.setInputCloud (cloud_origin);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud_origin,"intensity");
    origin_viewer->addPointCloud(cloud_origin, intensity_color, "origin");
    origin_viewer->registerAreaPickingCallback(area_pick_callback, (void*)&cloud_origin);
 
    while (!origin_viewer->wasStopped())
        origin_viewer->spin();
    pcl::io::savePCDFileASCII("../selected.pcd", *selected_area);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_selected(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile("../selected.pcd", *cloud_selected)) {
        std::cerr << "cannot open file selected.pcd" << std::endl;
        return 0;
    }
    // if (pcl::io::loadPLYFile("../selected.ply", *cloud_selected)) {
    //     std::cerr << "cannot open file selected.ply" << std::endl;
    //     return 0;
    // }
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> _intensity_color(cloud_selected,"intensity");
    pcl::visualization::PCLVisualizer selected_viewer("selected_viewer");
    selected_viewer.addPointCloud(cloud_selected, _intensity_color, "selected");
    
    while (!selected_viewer.wasStopped())
        selected_viewer.spin();

    return 0;
}