#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_selected(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *cloud_selected);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> voxel_sampler;
        voxel_sampler.setInputCloud(cloud_selected);
        voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_sampler.filter(*cloud_downsampled);
    pcl::io::savePCDFileASCII("../downsampled.pcd", *cloud_downsampled);
    
    pcl::visualization::CloudViewer downsampled_viewer("downsampled cloud");
    while (!downsampled_viewer.wasStopped())
        downsampled_viewer.showCloud(cloud_downsampled);


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud (cloud_downsampled);
        sor.setMeanK (100);
        sor.setStddevMulThresh (0.5);
        sor.filter (*cloud_filtered);
    // *cloud_filtered = *cloud_downsampled;
    pcl::io::savePCDFileASCII("../filtered.pcd", *cloud_filtered);

    pcl::visualization::CloudViewer filtered_viewer("filtered cloud");
    while (!filtered_viewer.wasStopped())
        filtered_viewer.showCloud(cloud_filtered);
 

	pcl::ModelCoefficients coefficients; // 模型系数
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); // 内点索引
    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
        segmentation.setModelType(pcl::SACMODEL_PLANE); // 平面模型
        segmentation.setMethodType(pcl::SAC_PROSAC);// 迭代算法
        segmentation.setMaxIterations(1000);// 最大迭代次数
        segmentation.setDistanceThreshold(0.08);// 最大误差
        segmentation.setInputCloud(cloud_filtered);
        segmentation.segment(*inliers, coefficients); // 结果 *inliers内点索引，coe是模型系数
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract; // 从点云中提取内点
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_segmented);
    // *cloud_segmented = *cloud_downsampled;
    pcl::io::savePCDFileASCII("../segmented.pcd", *cloud_segmented);

    pcl::visualization::CloudViewer segmented_viewer("segmented cloud");
    while (!segmented_viewer.wasStopped())
        segmented_viewer.showCloud(cloud_segmented);

    return 0;
}