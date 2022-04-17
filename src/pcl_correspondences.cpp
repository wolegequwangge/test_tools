#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation.h>

#define ToRad 0.01745328

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZINormal NormalT;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<NormalT> NormalCloudNT;

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cerr << "please enter pcd path!" << std::endl;
        return 0;
    }

    PointCloudT::Ptr source(new PointCloudT);
    PointCloudT::Ptr target(new PointCloudT);

    pcl::io::loadPCDFile(argv[1], *source);
    pcl::io::loadPCDFile(argv[2], *target);

    NormalCloudNT::Ptr normals(new NormalCloudNT);

    pcl::Correspondences all_correspondences;
    // pcl::registration::CorrespondenceEstimationBackProjection<PointT, PointT, NormalT, float> cebp;
    // pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> crs;

    // cebp

    pcl::registration::CorrespondenceEstimation<PointT, PointT> ce;
    ce.setInputSource(source);
    ce.setInputTarget(target);
    // ce.setPointRepresentation()
    // ce.determineCorrespondences(all_correspondences);
    ce.determineReciprocalCorrespondences(all_correspondences);

    std::cout << source->size() << std::endl;
    std::cout << all_correspondences.size() << std::endl;
    // for(pcl::Correspondences::iterator it = all_correspondences.begin(); it!=all_correspondences.end(); it++)
    // if((it - all_correspondences.begin())%1000 == 0) std::cout << *it << std::endl;
    // std::cout << *it << std::endl;

    // pcl::iter

    pcl::visualization::PCLVisualizer viewer("correspondences");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_source_color_h(source, 20, 180, 20);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_target_color_h(target, 180, 20, 20);
    viewer.addPointCloud(source, cloud_source_color_h, "source");
    viewer.addPointCloud(target, cloud_target_color_h, "target");

    viewer.addCorrespondences<PointT>(source, target, all_correspondences);

    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }

    return 0;
}