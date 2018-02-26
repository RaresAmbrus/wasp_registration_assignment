#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>

#include <fstream>

#include <pcl/registration/icp.h>

#include "utils.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

namespace fs = ::boost::filesystem;
using namespace std;
using namespace cv;

void printUsage(){
    std::cout<<"Usage: icp_registration target_folder"<<std::endl;
    std::cout<<"Default folder: /home/wasp/wasp_registration/data/wasp_registration/"<<std::endl;
    std::cout<<"Note: this will look for all files matching *.pcd in the target_folder"<<std::endl;
}

int main(int argc, char** argv){
    std::string clouds_folder;
    if (argc < 2){
        printUsage();
        clouds_folder = "/home/wasp/wasp_registration/data/wasp_registration/";
    } else {
        clouds_folder = argv[1];
    }
    bool bVerbose = false;

    /****************************** LOAD AND VISUALIZE DATA ***************************************/
    if (!fs::is_directory(clouds_folder)){
        printUsage();
        exit(-1);
    }

    int vp_1, vp_2, vp_3;
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "visualize_clouds");
    pg->setCameraPosition(-0.0979963,-6.23402,-3.88424,0.242764,0.415186,1.60075,0.235308,-0.625621,0.743794); // top down view
    pg->addCoordinateSystem(1.0);

    std::vector<CloudPtr> all_views = wasp_registration_utils::load_pcds<PointType>(clouds_folder);
    for (int i=0; i<all_views.size(); ++i){
        stringstream ss; ss<<"Cloud";ss<<i;
        cout<<"Displaying point cloud "<<i<<endl;
        pg->addPointCloud(all_views[i], ss.str());
        (bVerbose == true) ? pg->spin() : pg->spinOnce();
        pg->removeAllPointClouds();
    }

    // Create viewports
    pg->createViewPort (0.0, 0, 0.5, 0.5, vp_1); // Bottom left viewport
    pg->createViewPort (0.5, 0, 1.0, 0.5, vp_2); // Bottom right viewport
    pg->createViewPort (0.0, 0.5, 1.0, 1.0, vp_3); // Top viewport
    pg->addCoordinateSystem(1.0, vp_1);
    pg->addCoordinateSystem(1.0, vp_2);
    pg->addCoordinateSystem(1.0, vp_3);

    pg->addText("Registration results", 10, 10,20,1,1,1,"vp_3_text",vp_3);
    pg->addText("Before reg.", 10, 10,20,1,1,1,"vp_1_text",vp_1);
    pg->addText("After reg.", 10, 10,20,1,1,1,"vp_2_text",vp_2);

    /****************************** ICP CLOUDS ***************************************/
    cout<<"Registering point clouds with ICP. Accumulated registration results on top. Clouds before registration - bottom left, and after registration - bottom right."<<endl;

    // First step - downsample and filter (i.e. remove points with invalid values) the point clouds
    std::vector<CloudPtr> filtered_views; /// Downsample the input point clouds and store them in this array
    for (int i=0; i<all_views.size(); ++i){
        /** ----------------- 1. YOUR CODE HERE -------------------------
          * Fill in the vector filtered_views.
          * Note: for downsampling and filtering you should implement and call the method wasp_registration_utils::downsample_and_filter_cloud from utils.h
          * Optionally: visualize the effect of downsampling.
          *
          */

    }

    // Second step - perform registration using ICP
    // Add the first cloud to the visualizer
    pg->addPointCloud(filtered_views[0], "cloud_0",vp_3);

    Eigen::Matrix4f Tt = Eigen::Matrix4f::Identity(); /// Use this variable to keep track of the transform of the current cloud to the global frame of reference.
    vector<Eigen::Matrix4f> view_transforms; /// Use this vector to store the transform to a global frame of reference where all the clouds are registered with each other.
    view_transforms.push_back(Tt); /// By convention, the transform of the first cloud is identity.
    for (int i=0; i<all_views.size() - 1; ++i){
        cout<<"Registering cloud "<<i<<" and cloud "<<i+1<<endl;

        // visualize clouds before registration
        pcl::visualization::PointCloudColorHandlerCustom<PointType> target_h (filtered_views[i], 255, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<PointType> source_h (filtered_views[i+1], 0, 255, 0);
        pg->addPointCloud(filtered_views[i], target_h, "cloud1_before", vp_1);
        pg->addPointCloud(filtered_views[i+1], source_h, "cloud2_before", vp_1);
        (bVerbose == true) ? pg->spin() : pg->spinOnce();

        // apply ICP iteratively
        /// Use this variable to store the transform between cloud i+1 and cloud i.
        /// (e.g. cloud_i = cloud_(i+1) * Ti
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
        /// Use this variable to store the result of registering cloud i+1 with cloud i
        CloudPtr reg_result(new Cloud);

        /** ----------------- 2. YOUR CODE HERE -------------------------
          * Use ICP to align clouds i and i+1
          */




        /** ----------------- 3. YOUR CODE HERE -------------------------
          * Update the variable Tt so that it contains the transform which brings the cloud (i+1) into the global frame of reference.
          * (i.e. you should make use of the variable Ti which stores the intermediate transformation between cloud (i+1) and cloud i
          */


        // Store cloud transform
        view_transforms.push_back(Tt);

        // Visualize clouds i and i+1 (after registration) in the visualizer (bottom right window)
        pcl::visualization::PointCloudColorHandlerCustom<PointType> source_h_after (reg_result, 0, 255, 0);
        pg->removeAllPointClouds(vp_2);
        pg->addPointCloud(filtered_views[i], target_h, "cloud1_after", vp_2);
        pg->addPointCloud(reg_result, source_h_after, "cloud2_after", vp_2);
        pg->spinOnce();

        // Transform the cloud i+1 into the global frame of reference and add it to the visualizer (top window)
        pcl::transformPointCloud (*filtered_views[i+1], *reg_result, Tt); /// transform cloud i+1 into the global frame of reference.
        stringstream ss_latest; ss_latest<<"cloud_"<<i+1;
        pg->addPointCloud(reg_result, ss_latest.str(),vp_3);

        cout<<"Clouds after registration (right)"<<endl;
        (bVerbose == true) ? pg->spin() : pg->spinOnce();
        pg->removeAllPointClouds(vp_1);
        pg->removeAllPointClouds(vp_2);

    }

    return 0;
}
