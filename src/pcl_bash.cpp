#include <iostream>
#include <vector>
#include <string>
#include <ostream>
#include <iomanip>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    PCL_INFO("Rotation matrix :\n");
    PCL_INFO("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    PCL_INFO("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    PCL_INFO("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    PCL_INFO("Translation vector :\n");
    PCL_INFO("t = < %6.3f %6.3f %6.3f > \n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

inline std::string num2str(int i) {
    std::ostringstream ostr;
    ostr << std::setfill('0') << std::setw(4) << std::to_string(i);
    std::string res = "/home/wjq/data/datasets/pcd/" + ostr.str() + ".pcd";
    return res;
}

void filter_non_point(PointCloudT::Ptr &cloud_src){
    std::cout << " cloud filter in: " << cloud_src->size() << std::endl;
        
    if(cloud_src->empty()) {
        std::cout << " cloud filter output: " << cloud_src->size() << std::endl;
        return;
    } else {
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_src);
        std::vector< int > indices;
        for(PointCloudT::iterator it = cloud_src->begin(); it != cloud_src->end();it++) {
            if(it->intensity != 0) {
                it->z = 0;
                indices.push_back(it - cloud_src->begin());
            }// else ++it;
        }
        extract.setIndices (boost::make_shared<std::vector<int>>(indices));
        extract.setNegative (false);//如果设为true,可以提取指定index之外的点云
        extract.filter (*cloud_src);

        // pcl::VoxelGrid<pcl::PointXYZI> voxel_sampler;
        // voxel_sampler.setInputCloud(cloud_src);
        // voxel_sampler.setLeafSize(0.05f, 0.05f, 0.05f);
        // voxel_sampler.filter(*cloud_src);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud (cloud_src);
        sor.setMeanK (100);
        sor.setStddevMulThresh (0.5);
        sor.filter (*cloud_src);

        std::cout << " cloud filter output: " << cloud_src->size() << std::endl;
        return;
    }
}

bool next_accum = false;
bool next_ndt = false;
bool next_icp = false;
bool save = false;
bool fix = false;

void accumEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
    if (event.getKeySym() == "space" && event.keyDown()) next_accum = true;
}

void ndtEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
    if (event.getKeyCode() == 'n' && event.keyDown()) next_ndt = true;
}

void icpEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
    if (event.getKeyCode() == 'i' && event.keyDown()) next_icp = true;
}

void fixEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
    if (event.getKeyCode() == 'f' && event.keyDown()) fix = true;
}

void saveEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
    if (event.getKeyCode() == 's' && event.keyDown()) save = true;
}


int main(int argc, char** argv){
    if (argc != 2) {
        std::cerr << "please enter img path!" << std::endl;
        return 0;
    }
    int pcd_nums = std::stoi(argv[1]);

    PointCloudT::Ptr cloud_accum(new PointCloudT);
    pcl::io::loadPCDFile("/home/wjq/workspace/pcl_test/src/accum.pcd", *cloud_accum);
    std::cout << "cloud cur: /home/wjq/workspace/pcl_test/src/accum.pcd size: " << cloud_accum->size() << std::endl;

    // filter_non_point(cloud_accum);
    PointCloudT::Ptr cloud_cur(new PointCloudT);
    // PointCloudT::Ptr cloud_ndt(new PointCloudT);

    int ndt_count = 1;
    int icp_count = 1;
    int pcd_count = 37;

    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0.15, -0.10, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(0.1);
    ndt.setMaximumIterations(1);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setTransformationEpsilon(0.01);
    icp.setEuclideanFitnessEpsilon(0.01);
    icp.setMaxCorrespondenceDistance(0.1);
    icp.setMaximumIterations(1);

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> accum_cloud_color_h(cloud_accum, "intensity");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> reg_cloud_color_h(180, 20, 20);

    pcl::visualization::PCLVisualizer viewer("accumulation");

    // viewer.addPointCloud(cloud_init, accum_cloud_color_h, "0000");
    viewer.registerKeyboardCallback(&accumEventOccurred, (void *)NULL);
    viewer.registerKeyboardCallback(&ndtEventOccurred, (void *)NULL);
    viewer.registerKeyboardCallback(&icpEventOccurred, (void *)NULL);
    viewer.registerKeyboardCallback(&fixEventOccurred, (void *)NULL);
    viewer.registerKeyboardCallback(&saveEventOccurred, (void *)NULL);

    
    viewer.addPointCloud(cloud_accum, accum_cloud_color_h, "accum");
    // viewer.addPointCloud(cloud_ndt, reg_cloud_color_h, "ndt");

    while (!viewer.wasStopped()) {
        viewer.spinOnce();

        if (pcd_count < pcd_nums && next_accum == true) {

            std::string cur_name(num2str(pcd_count));
            pcl::io::loadPCDFile(cur_name, *cloud_cur);
            std::cout << "cloud cur: " << cur_name << " size: " << cloud_cur->size() << std::endl;

            PointCloudT::Ptr cloud_filter_cur(new PointCloudT);
            filter_non_point(cloud_cur);
                
            viewer.addPointCloud(cloud_cur, reg_cloud_color_h, "cur");
            pcd_count++;
        }
        next_accum = false;
 
        if (next_ndt == true) {
            // Setting point cloud to be aligned to.
            ndt.setInputTarget(cloud_accum);
            // Setting point cloud to be aligned.
            ndt.setInputSource(cloud_cur);
            // PointCloudT::Ptr cloud_ndt(new PointCloudT);
            // if(ndt_count == 1) ndt.align(*cloud_cur, init_guess);
            if(ndt_count == 1) ndt.align(*cloud_cur);
            else ndt.align(*cloud_cur);
            ndt_count++;
            std::cout << "cloud_ndt size: " << cloud_cur->size() << std::endl;
            if (ndt.hasConverged()) {
                printf("\033[11A"); // Go up 11 lines in terminal output.
                printf("\nNDT has converged, score is %+.0e\n", ndt.getFitnessScore());
                std::cout << "\nNDT transformation " << ndt_count << " : ndt_cloud -> input_cloud" << std::endl;
                print4x4Matrix(ndt.getFinalTransformation().cast<double>());

                viewer.removePointCloud("cur");

                viewer.addPointCloud(cloud_cur, reg_cloud_color_h, "cur");

            } else {
                PCL_ERROR("\nNDT has not converged.\n");
                return (-1);
            }
        }
        next_ndt = false;

        if (next_icp == true) {
            // Setting point cloud to be aligned to.
            icp.setInputTarget(cloud_accum);
            // Setting point cloud to be aligned.
            icp.setInputSource(cloud_cur);
            // PointCloudT::Ptr cloud_ndt(new PointCloudT);
            // if(ndt_count == 1) icp.align(*cloud_cur, init_guess);
            if(ndt_count == 1) icp.align(*cloud_cur);        
            else icp.align(*cloud_cur);
            icp_count++;
            std::cout << "cloud_icp size: " << cloud_cur->size() << std::endl;
            if (icp.hasConverged()) {
                printf("\033[11A"); // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << icp_count << " : icp_cloud -> input_cloud" << std::endl;
                print4x4Matrix(icp.getFinalTransformation().cast<double>());

                viewer.removePointCloud("cur");

                viewer.addPointCloud(cloud_cur, reg_cloud_color_h, "cur");

            } else {
                PCL_ERROR("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_icp = false;

        if(fix == true) {
            *cloud_accum += *cloud_cur;
            std::cout << "cloud_accum size: " << cloud_accum->size() << std::endl;
            viewer.removePointCloud("cur");
            viewer.removePointCloud("accum");
            viewer.addPointCloud(cloud_accum, accum_cloud_color_h, "accum");
            ndt_count = 1;
            icp_count = 1;
            cloud_cur->clear();
        }
        fix = false;
        
        if(save == true) {
            pcl::io::savePCDFileASCII("accum.pcd", *cloud_accum);
            std::cout << "accum.pcd size : " << cloud_accum->size() << std::endl;
        }
        save = false;
    }    
    return 0;
}