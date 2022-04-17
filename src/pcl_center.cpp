#include <iostream>
#include <vector>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
 
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr selected_area(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointIndices _indices;
boost::shared_ptr<pcl::visualization::PCLVisualizer> origin_viewer(new pcl::visualization::PCLVisualizer("origin_viewer"));

// int k = 0; 
void area_pick_callback(const pcl::visualization::AreaPickingEvent& event, void* args){
    std::vector< int > indices;
    if (event.getPointsIndices(indices)==-1)
        return;
    _indices.indices = indices;
    for (int i = 0; i < indices.size(); ++i)
        selected_area->points.push_back(cloud_origin->points.at(indices[i]));

    // selected_area->width = 1;
    // selected_area->height = selected_area->points.size();

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
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud_origin,"intensity");
    origin_viewer->addPointCloud(cloud_origin, intensity_color, "origin");
    origin_viewer->registerAreaPickingCallback(area_pick_callback, (void*)&cloud_origin);
 
    while (!origin_viewer->wasStopped())
        origin_viewer->spin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_non_selected(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extractor;
    extractor.setInputCloud(cloud_origin);
    extractor.setIndices(boost::make_shared<pcl::PointIndices>(_indices));

    // extractor.setNegative(false); //true removes the indices, false leaves only the indices
    // extractor.filter(*out_only_ground_cloud);

    extractor.setNegative(true); //true removes the indices, false leaves only the indices
    extractor.filter(*cloud_non_selected);
    pcl::io::savePCDFileASCII("cornor.pcd", *cloud_non_selected);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    double x(0),y(0),z(0);
    size_t _size(cloud_non_selected->size());
#pragma omp for
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = cloud_non_selected->points.begin(); pt < cloud_non_selected->points.end(); pt++){
       x+=pt->x;
       y+=pt->y;
       z+=pt->z; 
    }
    std::cout << "x: " << x/_size << " "
              << "y: " << y/_size << " "
              << "z: " << z/_size << std::endl;

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> __intensity_color(cloud_non_selected,"intensity");
    pcl::visualization::PCLVisualizer non_selected_viewer("non_selected_viewer");
    non_selected_viewer.addPointCloud(cloud_non_selected, __intensity_color, "non_selected");
    
    while (!non_selected_viewer.wasStopped())
        non_selected_viewer.spin();

    return 0;
}