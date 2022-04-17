#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

#define PI 3.14159265
// typedef pcl::Normal NormalT;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZINormal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloudT;
PointCloudT::Ptr cloud_input(new PointCloudT);
NormalCloudT::Ptr cloud_normal (new NormalCloudT);
PointCloudT::Ptr cloud_diff(new PointCloudT);
PointCloudT::Ptr cloud_selected(new PointCloudT);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_ptr(new pcl::visualization::PCLVisualizer("cloud"));

void point_pick_callback(const pcl::visualization::PointPickingEvent& event, void* args) {
    int idx = event.getPointIndex();
    if (idx == -1) 
        return;
    pcl::PointXYZI current_point = cloud_diff->points.at(idx);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> white_ch(cloud_selected, 255, 255, 255);
    cloud_selected->points.push_back(current_point);
    
    viewer_ptr->removePointCloud("clicked_points"); // 移除"clicked_points"点云，用于多次选择
    viewer_ptr->addPointCloud(cloud_selected, white_ch, "clicked_points");
    viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points"); // 渲染选中点，设置PCL_VISUALIZER_POINT_SIZE为10
    std::cout << 
        " x: " << current_point.x << 
        " y: " << current_point.y << 
        " z: " << current_point.z << 
        " i: " << current_point.intensity << 
    std::endl;
}
int main(int argc, char** argv){

    pcl::io::loadPCDFile(argv[1], *cloud_input);
    pcl::NormalEstimationOMP<PointT, NormalT> ne;
    ne.setNumberOfThreads(16);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.5);
    ne.setInputCloud(cloud_input);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    ne.compute (*cloud_normal);

    viewer_ptr->addCoordinateSystem();
    double nx(0.0237744);
    double ny(-0.0141324);
    double nz(0.999617);
#pragma omp for
    for(size_t i=0; i<cloud_input->size(); i++) {
        PointT p = cloud_input->points[i];
        p.intensity = acos(cloud_normal->points[i].normal_x * nx + cloud_normal->points[i].normal_y * ny + cloud_normal->points[i].normal_z * nz);
        p.intensity *= (180 / PI);
        // p.intensity = cloud_normal->points[i].curvature;
        cloud_diff->points.push_back(p);
    }

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity_color(cloud_diff,"intensity");
    viewer_ptr->addPointCloud(cloud_diff, intensity_color, "origin");
    // viewer_ptr->addPointCloudNormals<PointT, NormalT>(cloud_diff, cloud_normal, 1, 0.1, "cloud_normal");
    viewer_ptr->registerPointPickingCallback(point_pick_callback);

    while (!viewer_ptr->wasStopped()) viewer_ptr->spin();
    return 0;

}