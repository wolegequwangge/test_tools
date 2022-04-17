#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

#include <iostream>
#include <chrono>

// typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointT;

using namespace std;

pcl::visualization::PCLPlotter plotter;
//pcl::FPFHEstimation<pcl::PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
pcl::FPFHEstimationOMP<PointT, pcl::PointNormal, pcl::FPFHSignature33> fpfh_omp;
pcl::FPFHEstimation<PointT, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

pcl::PFHEstimation<PointT, pcl::PointNormal, pcl::PFHSignature125> pfh;
pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

// structure used to pass arguments to the callback function
struct callback_args {
    pcl::PointCloud<PointT>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

// callback function
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    plotter.clearPlots();
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.clear();
    data->clicked_points_3d->points.push_back(current_point);

    // Draw clicked points in red:
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

    int num = event.getPointIndex();

    pcl::PFHSignature125 descriptor;
    descriptor = pfhs->points[num];
    std::cout << " -- pfh for point "<< num << ":\n" << descriptor << std::endl;
    plotter.addFeatureHistogram<pcl::PFHSignature125>(*pfhs, "pfh", num);

    // pcl::FPFHSignature33 descriptor;
    // descriptor = fpfhs->points[num];
    // std::cout << " -- fpfh for point "<< num << ":\n" << descriptor << std::endl;
    // plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh", num);
    
    plotter.plot();
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf(" -- Usage: %s <pointcloud file>\n", argv[0]);
        return -1;
    }

    bool display = true;
    bool downSampling = true;

    // load pcd/ply point cloud
    pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT>()); // 模型点云
    if (pcl::io::loadPCDFile(argv[1], *model) < 0) {
        std::cerr << "Error loading model cloud." << std::endl;
        return -1;
    }
    std::cout << "Cloud size: " << model->points.size() << std::endl;

    if (downSampling) {
        // create the filtering object
        std::cout << "Number of points before downSampling: " << model->points.size() << std::endl;
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(model);
        sor.setLeafSize(0.01, 0.01, 0.01);
        sor.filter(*model);
        std::cout << "Number of points after downSampling: " << model->points.size() << std::endl;
    }

    //  Normal estimation
    auto t1 = chrono::steady_clock::now();

    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setNumberOfThreads(12);
    ne.setInputCloud(model);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1);
    // ne.setKSearch(100);
    ne.compute(*normals);
    auto t2 = chrono::steady_clock::now();
    auto dt = chrono::duration_cast<chrono::duration<double> >(t2 - t1).count();
    cout << "Time cost of Normal estimation: " << dt << endl;

    // fpfh or fpfh_omp
    pfh.setInputCloud(model);
    pfh.setInputNormals(normals);
    pfh.setSearchMethod(tree);
    pfh.setRadiusSearch(0.1);
    pfh.compute(*pfhs);
    // fpfh_omp.setNumberOfThreads(12);
    // fpfh_omp.setInputCloud(model);
    // fpfh_omp.setInputNormals(normals);
    // fpfh_omp.setSearchMethod(tree);
    // fpfh_omp.setRadiusSearch(0.1);
    // fpfh_omp.compute(*fpfhs);
    auto t3 = chrono::steady_clock::now();
    dt = chrono::duration_cast<chrono::duration<double> >(t3 - t2).count();
    cout << "Time cost of FPFH estimation: " << dt << endl;

    if (display) {
        // plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh",100);
        plotter.addFeatureHistogram<pcl::PFHSignature125>(*pfhs, "pfh",100);

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<PointT>(model, "model");
        viewer->addPointCloudNormals<PointT, pcl::PointNormal>(model, normals, 10, 0.05, "normals");  // display every 1 points, and the scale of the arrow is 10

        // Add point picking callback to viewer:
        struct callback_args cb_args;
        pcl::PointCloud<PointT>::Ptr clicked_points_3d(new pcl::PointCloud<PointT>);
        cb_args.clicked_points_3d = clicked_points_3d;
        cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
        viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
        }
    }

    return 0;
}