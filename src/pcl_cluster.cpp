#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZINormal NormalT;

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "please enter 'pcd path' !" << std::endl;
        return 0;
    }
    PointCloudT::Ptr cloud_raw(new PointCloudT());
    pcl::io::loadPCDFile(argv[1], *cloud_raw);

    // pcl::visualization::CloudViewer raw_viewer("raw_cloud");
    // while (!raw_viewer.wasStopped()) raw_viewer.showCloud(cloud_raw);  
    
    // PointCloudT::Ptr cloud_sampler(new PointCloudT());
    // pcl::VoxelGrid<pcl::PointXYZI> voxel_sampler;
    // voxel_sampler.setInputCloud(cloud_raw);
    // voxel_sampler.setLeafSize(0.05f, 0.05f, 0.05f);
    // voxel_sampler.filter(*cloud_sampler);
    // // pcl::visualization::CloudViewer sampler_viewer("sampler_cloud");
    // // while (!sampler_viewer.wasStopped()) sampler_viewer.showCloud(cloud_sampler);
    
    // PointCloudT::Ptr cloud_sor(new PointCloudT());
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    //     sor.setInputCloud (cloud_sampler);
    //     sor.setMeanK (250);
    //     sor.setStddevMulThresh (0.5);
    //     sor.filter (*cloud_sor);
    // pcl::visualization::CloudViewer sor_viewer("sor_cloud");
    // while (!sor_viewer.wasStopped()) sor_viewer.showCloud(cloud_sor);
    // pcl::io::savePCDFileBinary("sor.pcd", *cloud_sor);

    pcl::NormalEstimationOMP<pcl::PointXYZI, NormalT> ne;
    ne.setNumberOfThreads(16);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
    ne.setSearchMethod (tree);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    // ne.setInputCloud (cloud_sor);
    ne.setInputCloud(cloud_raw);

    ne.setRadiusSearch (5);
    pcl::PointCloud<NormalT>::Ptr small_cloud_normals (new pcl::PointCloud<NormalT>);
    ne.compute (*small_cloud_normals);

    pcl::visualization::PCLVisualizer small_cloud_normals_viewer("small_cloud");
    small_cloud_normals_viewer.addPointCloudNormals<pcl::PointXYZI, NormalT>(cloud_raw, small_cloud_normals, 1, 0.1);
    while (!small_cloud_normals_viewer.wasStopped())
    small_cloud_normals_viewer.spin();


    ne.setRadiusSearch (10);
    pcl::PointCloud<NormalT>::Ptr large_cloud_normals (new pcl::PointCloud<NormalT>);
    ne.compute (*large_cloud_normals);
    
    pcl::visualization::PCLVisualizer large_cloud_normals_viewer("large_cloud");
    large_cloud_normals_viewer.addPointCloudNormals<pcl::PointXYZI, NormalT>(cloud_raw, large_cloud_normals, 1, 0.1);
    while (!large_cloud_normals_viewer.wasStopped())
    large_cloud_normals_viewer.spin();

    // Create output cloud for DoN results
    pcl::PointCloud<NormalT>::Ptr doncloud (new pcl::PointCloud<NormalT>);
    // pcl::copyPointCloud<PointT, NormalT>(*cloud_sor, *doncloud);
    pcl::copyPointCloud<PointT, NormalT>(*cloud_raw, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointT, NormalT, NormalT> don;
    // don.setInputCloud (cloud_sor);
    don.setInputCloud(cloud_raw);
    don.setNormalScaleLarge (small_cloud_normals);
    don.setNormalScaleSmall (large_cloud_normals);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);
    cout << "Calculating DoN done " << endl;

    pcl::visualization::PCLVisualizer doncloud_viewer("doncloud");
    // normal_viewer.addPointCloudNormals<pcl::PointXYZI, NormalT>(cloud_sor, doncloud);
    doncloud_viewer.addPointCloudNormals<pcl::PointXYZI, NormalT>(cloud_raw, doncloud, 1, 0.1);
    while (!doncloud_viewer.wasStopped())
    doncloud_viewer.spin();

    double threshold = 0.5;

    pcl::ConditionOr<NormalT>::Ptr range_cond (new pcl::ConditionOr<NormalT> ());
    range_cond->addComparison (pcl::FieldComparison<NormalT>::ConstPtr (new pcl::FieldComparison<NormalT> ("curvature", pcl::ComparisonOps::GT, threshold)));
    // Build the filter
    pcl::ConditionalRemoval<NormalT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (doncloud);
    pcl::PointCloud<NormalT>::Ptr doncloud_filtered (new pcl::PointCloud<NormalT>);
    // Apply filter
    condrem.filter (*doncloud_filtered);
    std::cout << "doncloud_filtered size is " << doncloud_filtered->size() << std::endl;

    // doncloud = doncloud_filtered;

    pcl::search::KdTree<NormalT>::Ptr segtree (new pcl::search::KdTree<NormalT>);
    segtree->setInputCloud (doncloud_filtered);
    
    pcl::visualization::PCLVisualizer doncloud_filtered_viewer("doncloud_filtered");
    // normal_viewer.addPointCloudNormals<pcl::PointXYZI, NormalT>(cloud_sor, doncloud);
    doncloud_filtered_viewer.addPointCloudNormals<pcl::PointXYZI, NormalT>(cloud_raw, doncloud_filtered, 1, 0.1);
    while (!doncloud_filtered_viewer.wasStopped())
    doncloud_filtered_viewer.spin();

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<NormalT> ec;

    double segradius = 1.5;

    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud_filtered);
    ec.extract (cluster_indices);
    
    std::cout << "cluster_indices size is " << cluster_indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl::PointCloud<NormalT>::Ptr cloud_cluster_don (new pcl::PointCloud<NormalT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZHSV p;
            pcl::PointXYZRGB p_rgb;
            p.x = doncloud_filtered->points[*pit].x;
            p.y = doncloud_filtered->points[*pit].y;
            p.z = doncloud_filtered->points[*pit].z;
            p.h = 360 * (it-cluster_indices.begin()) / cluster_indices.size();
            std::cout << "(it-cluster_indices.begin()) is " << (it-cluster_indices.begin()) << std::endl;
            std::cout << "cluster_indices.size() is " << cluster_indices.size() << std::endl;
            std::cout << "p.h is " << p.h << std::endl;
            p.s = 0.8;
            p.v = 0.8;
            pcl::PointXYZHSVtoXYZRGB(p, p_rgb);
            _cloud_cluster->points.push_back(p_rgb);
        }
        _cloud_cluster->width = int (_cloud_cluster->points.size());
        _cloud_cluster->height = 1;
        _cloud_cluster->is_dense = true;
        *cloud_cluster += *_cloud_cluster;

    }
    pcl::visualization::CloudViewer color_viewer("color_cloud");
    while (!color_viewer.wasStopped()) color_viewer.showCloud(cloud_cluster);
    
    // pcl::io::savePCDFileBinary("cluster.pcd", *cloud_cluster);

    // pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    // tree->setInputCloud(cloud_sor);

    // PointCloudT::Ptr cloud_bil(new PointCloudT());
    // std::cout << "bilateral filter start" << std::endl;
    // pcl::BilateralFilter<PointT> bil;
    // bil.setInputCloud(cloud_sor);
    // bil.setStdDev(std::stof(argv[2]));
    // bil.setHalfSize(std::stof(argv[3]));
    // bil.setSearchMethod(tree);
    // bil.filter(*cloud_bil);
    // pcl::visualization::CloudViewer bil_viewer("bil_cloud");
    // while (!bil_viewer.wasStopped()) bil_viewer.showCloud(cloud_bil);

    return 0;
}