#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/filters/statistical_outlier_removal.h>


// Mutex: //
// boost::mutex cloud_mutex;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr selected_points(new pcl::PointCloud<pcl::PointXYZI>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> segmented_viewer(new pcl::visualization::PCLVisualizer("segmented_viewer"));
  
void point_pick_callback(const pcl::visualization::PointPickingEvent& event, void* args) {
    int idx = event.getPointIndex();
    if (idx == -1) 
        return;
    pcl::PointXYZI current_point = cloud_segmented->points.at(idx);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> white_ch(selected_points, 255, 255, 255);
    selected_points->points.push_back(current_point);
    
    segmented_viewer->removePointCloud("clicked_points"); // 移除"clicked_points"点云，用于多次选择
    segmented_viewer->addPointCloud(selected_points, white_ch, "clicked_points");
    segmented_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points"); // 渲染选中点，设置PCL_VISUALIZER_POINT_SIZE为10
    std::cout << 
        " x: " << current_point.x << 
        " y: " << current_point.y << 
        " z: " << current_point.z << 
        " i: " << current_point.intensity << 
    std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr diff_cloud_intensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input, int max_threshold_value, int min_threshold_value) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    for (pcl::PointCloud<pcl::PointXYZI>::iterator pt = input->points.begin(); pt < input->points.end(); pt++)
        if (pt->intensity < max_threshold_value && pt->intensity > min_threshold_value)
            output->points.push_back(*pt);
    return output;
}

int main(int argc,char** argv) {
  
    if (pcl::io::loadPCDFile(argv[1], *cloud_segmented)) {
        std::cerr << "cannot open file segmented.pcd" << std::endl;
        return 0;
    }
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_color(cloud_segmented, "intensity");
    segmented_viewer->addPointCloud(cloud_segmented, intensity_color, "cloud_segmented");
    // cloud_mutex.lock();
    segmented_viewer->registerPointPickingCallback(point_pick_callback, (void*)&cloud_segmented);
    // std::cout << "shift+click to pick points" << std::endl;
    // // cloud_mutex.unlock();
    while (!segmented_viewer->wasStopped()) 
        segmented_viewer->spin();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_diff = diff_cloud_intensity(cloud_segmented, 500, 30);

    pcl::visualization::CloudViewer diff_viewer("diff cloud");
    while (!diff_viewer.wasStopped())
        diff_viewer.showCloud(cloud_diff);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud (cloud_diff);
        sor.setMeanK (10);
        sor.setStddevMulThresh (5);
        sor.filter (*cloud_final);
    pcl::io::savePCDFileASCII("../final.pcd", *cloud_final);

    pcl::visualization::CloudViewer filtered_viewer("final cloud");
    while (!filtered_viewer.wasStopped())
        filtered_viewer.showCloud(cloud_final);
    
    return 0;
}