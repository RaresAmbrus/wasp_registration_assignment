#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>

#include <pcl/registration/icp.h>

#include <fstream>
#include "utils.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

namespace fs = ::boost::filesystem;
using namespace std;
using namespace cv;

void printUsage(){
    std::cout<<"Usage: sift_registration target_folder"<<std::endl;
    std::cout<<"Default folder: /home/wasp/wasp_registration/data/wasp_registration/"<<std::endl;
    std::cout<<"Note: this will look for all files matching *.pcd in the target_folder"<<std::endl;


}

std::vector<int> get_corresponding_IDXs_for_current_IDx(const int& idx, const int& total_views){
    std::vector<int> idxs;
    if (idx+1 < total_views){
        idxs.push_back(idx+1);
    }

    return idxs;
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

    if (!fs::is_directory(clouds_folder)){
        printUsage();
        exit(-1);
    }

    /****************************** LOAD AND VISUALIZE CLOUDS ***************************************/

    int vp_1, vp_2, vp_3;
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "visualize_clouds");
    pg->setCameraPosition(-0.0979963,-6.23402,-3.88424,0.242764,0.415186,1.60075,0.235308,-0.625621,0.743794); // top down view
    pg->addCoordinateSystem(1);
    std::vector<CloudPtr> all_views = wasp_registration_utils::load_pcds<PointType>(clouds_folder);
    for (int i=0; i<all_views.size(); ++i){
        stringstream ss; ss<<"Cloud";ss<<i;
        cout<<"Displaying point cloud "<<i<<endl;
        pg->addPointCloud(all_views[i], ss.str());
        (bVerbose == true) ? pg->spin() : pg->spinOnce();
        pg->removeAllPointClouds();
    }

    // Create viewports
    pg->createViewPort (0.0, 0, 0.5, 0.5, vp_1);
    pg->createViewPort (0.5, 0, 1.0, 0.5, vp_2);
    pg->createViewPort (0.0, 0.5, 1.0, 1.0, vp_3);
    pg->addCoordinateSystem(1, vp_1);
    pg->addCoordinateSystem(1, vp_2);
    pg->addCoordinateSystem(1, vp_3);

    pg->addText("Registration results", 10, 10,20,1,1,1,"vp_3_text",vp_3);
    pg->addText("Before reg.", 10, 10,20,1,1,1,"vp_1_text",vp_1);
    pg->addText("Registration with all features", 10, 10,20,1,1,1,"vp_2_text",vp_2);

    cv::namedWindow("Original_SIFT_matches");
    cv::namedWindow("Filtered_SIFT_matches");

    // Downsample and filter (i.e. remove points with invalid values) the point clouds
    std::vector<CloudPtr> filtered_views;
    for (int i=0; i<all_views.size(); ++i){
        CloudPtr filtered_cloud = wasp_registration_utils::downsample_and_filter_cloud<PointType>(all_views[i],0.03, 0.03, 0.03);
        filtered_views.push_back(filtered_cloud);

        // Visualize downsampling effect
        cout<<"Visualizing cloud "<<i<<". Left=original; Right=downsampled."<<endl;
        pg->addPointCloud(all_views[i], "original", vp_1);
        pg->addPointCloud(filtered_views[i], "filtered", vp_2);
        (bVerbose == true) ? pg->spin() : pg->spinOnce();
        pg->removeAllPointClouds();
    }

    // add cloud_0 to the global registration viewport
    pg->addPointCloud(filtered_views[0], "cloud_0",vp_3);
    // global transformation matrix which will accumulate the incremental transformations
    std::vector<Eigen::Matrix4f> all_transforms(all_views.size());
    std::fill(all_transforms.begin(), all_transforms.end(), Eigen::Matrix4f::Identity());

    /// datastructure which will record correspondences as well as the indices of the clouds used to compute them; used for saving and loading the data;
    vector<wasp_registration_utils::CorrespondenceStructure> all_correspondences;
    for (int i=0; i<all_views.size()-1; ++i){
        /** ----------------- (PART IV) YOUR CODE HERE -------------------------
          * This method returns a set of indices corresponding to clouds which we want to register with the current cloud.
          * For cloud i, the default implementation returns cloud i+1.
          * You might want to modify this to return multiple values (e.g. given i, return (i-2,i-1,i+1,i+2).
          * This will be more relevant when defining the optimization in Part IV.
          */
        vector<int> idxs = get_corresponding_IDXs_for_current_IDx(i, all_views.size());
        for (int j : idxs){

            // downsample for visualization purposes
            pcl::visualization::PointCloudColorHandlerCustom<PointType> source_h (filtered_views[i], 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<PointType> target_h (filtered_views[j], 0, 255, 0);
            pg->addPointCloud(filtered_views[i], source_h, "cloud1_before", vp_1);
            pg->addPointCloud(filtered_views[j], target_h, "cloud2_before", vp_1);
            pg->spinOnce();

            // Create RGB and depth images
            pair<cv::Mat, cv::Mat> rgb_and_depth = wasp_registration_utils::create_RGB_and_depth_from_cloud(all_views[i]);
            pair<cv::Mat, cv::Mat> rgb_and_depth_next = wasp_registration_utils::create_RGB_and_depth_from_cloud(all_views[j]);

            std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
            /** ----------------- 1. YOUR CODE HERE -------------------------
              * Use OpenCV's SIFT feature detector to  detect SIFT features in the two RGB images.
              * Fill the results in the vectors keypoints_1 and keypoints_2
              */


            cv::Mat descriptors_1, descriptors_2;
            /** ----------------- 2. YOUR CODE HERE -------------------------
              * Use OpenCV's SIFT descriptor extractor to extract SIFT descriptors for each keypoint
              * Fill the results in the matrices descriptors_1 and descriptors_2
              */


            std::vector< DMatch > matches;
            /** ----------------- 3. YOUR CODE HERE -------------------------
              * Match the two sets of descriptors and return a set of matches
              * Fill the result in the vector matches
              */




            /** ----------------- 3. YOUR CODE HERE -------------------------
              * Optionally remove some of the matches mased on the minimum distance between the matches (i.e. dist < 2*min_dist)
              * This is known to discard some false correspondences.
              */


            /****************************** VISUALIZE MATCHES ***************************************/
            Mat img_matches;
            drawMatches( rgb_and_depth.first, keypoints_1, rgb_and_depth_next.first, keypoints_2,
                         matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            // Show detected matches
            cv::resize(img_matches, img_matches, cv::Size(640, 240));
            imshow( "Original_SIFT_matches", img_matches );
            (bVerbose == true) ? cv::waitKey(0) : cv::waitKey(200);


            // New point clouds storing the 3D points corresponding to the 2D features extracted
            // Important: make sure to remove points with invalid depth (e.g. nans or infs).
            CloudPtr c1(new Cloud);
            CloudPtr c2(new Cloud);
            /** ----------------- 4. YOUR CODE HERE -------------------------
              * Find the 3D points in the original point clouds which correspond to the 2D image correspondences you calculated.
              * Important: make sure to discard points with invalid depth (e.g. nans or infs).
              * Hint: you may want to use the method wasp_registration_utils::get_pcd_index_from_image_keypoint to convert between image keypoint coordinates and point cloud indices
              * Note that each element of type DMatch from the matches vector contains a query index and a train index.
              * Use these to access the correct keypoints from the vectors keypoints_1 and keypoints_2.
              * Store the resulting 3D points in the clouds c1 and c2.
              */



            Eigen::Matrix4f Tj = Eigen::Matrix4f::Identity();
            /** ----------------- 5. YOUR CODE HERE -------------------------
             * Use SVD to estimate the transformation between the clouds c2 and c1
             * Hint: PCL has a very nice class for doing this!
             * Store the result in the matrix Tj
             * Also, store the "global" transform of cloud j in the all_transforms vector
             * (Hint: you should make use of the already computed transform for cloud i).
             */


            /****************************** VISUALIZE TRANSFORMATION ***************************************/
            CloudPtr next_view_transformed(new Cloud);
            pcl::transformPointCloud (*filtered_views[j], *next_view_transformed, Tj);

            // Result of registration for clouds (i,i+1) in viewport 2
            pcl::visualization::PointCloudColorHandlerCustom<PointType> target_h_after (next_view_transformed, 0, 255, 0);
            pg->addPointCloud(filtered_views[i], source_h, "cloud1_after", vp_2);
            pg->addPointCloud(next_view_transformed, target_h_after, "cloud2_after", vp_2);
            pg->updateText("Registration with all features", 10, 10,"vp_2_text");
            pg->spinOnce();

            // Global result of registration in viewport 3
            CloudPtr global_result(new Cloud);
            pcl::transformPointCloud (*filtered_views[j], *global_result, all_transforms[j]);
            stringstream ss_latest; ss_latest<<"cloud_"<<j;
            pg->removePointCloud(ss_latest.str());
            pg->addPointCloud(global_result, ss_latest.str(),vp_3);
            cout<<"Press Q to filter SIFT correspondences and re-estimate the transformation"<<endl;
            (bVerbose == true) ? pg->spin() : pg->spinOnce();

            /****************************** FILTER CORRESPONDENCES ***************************************/
            pcl::CorrespondencesPtr pcl_correspondences(new pcl::Correspondences);
            /** ----------------- 6. YOUR CODE HERE -------------------------
             * Convert the openCV matches to a pcl::Correspondences data structure
             * Hint: you will need the same point cloud indices you computed when construction the clouds c1 and c2
             * Store the result in the pcl_correspondences data structure
             */



            pcl::Correspondences filtered_correspondences;
            /** ----------------- 6. YOUR CODE HERE -------------------------
             * Use the pcl::registration::CorrespondenceRejectorSampleConsensus to filter out matches
             * Store the result in the filtered_correspondences data structure.
             */


            // Update keypoints and 3D datastructures
            std::vector<cv::KeyPoint> filtered_keypoints_1, filtered_keypoints_2;
            std::vector< DMatch > filtered_matches;
            c1->points.clear();
            c2->points.clear();


            // Perform a test to make sure the consistency check returned valid results
            if (filtered_correspondences.size() > 10){

                int wait_time;
                (bVerbose == true) ? wait_time = 0 : wait_time = 200;
                // use the convenience function wasp_registration_utils::visualize_correspondences to visualize the correspondences
                wasp_registration_utils::visualize_correspondences(rgb_and_depth.first,
                                                                   rgb_and_depth_next.first,
                                                                   filtered_correspondences,
                                                                   "Filtered_SIFT_matches",
                                                                   wait_time);

                // Estimate transformation with the filtered set of correspondences
                /** ----------------- 7. YOUR CODE HERE -------------------------
                 * Fill in the clouds c1 and c2 using the data from the vector filtered_corresponcences.
                 * Use SVD to estimate the transformation between the clouds c2 and c1
                 * (this should be very similar to the code you already wrote earlier)
                 * Store the result in the transform Tj and don't forget to update the all_transforms vector
                 */

                /****************************** VISUALIZE RESULTS ***************************************/
                // first remove old data
                pg->removePointCloud("cloud2_after", vp_2);
                pg->removePointCloud(ss_latest.str(),vp_3);

                // transform clouds with updated results
                pcl::transformPointCloud (*filtered_views[j], *next_view_transformed, Tj);
                pg->addPointCloud(next_view_transformed, target_h_after, "cloud2_after", vp_2);
                pg->updateText("Registration with filtered features", 10, 10,"vp_2_text");

                pcl::transformPointCloud (*all_views[j], *global_result, all_transforms[j]);
                pg->addPointCloud(wasp_registration_utils::downsample_cloud<PointType>(global_result), ss_latest.str(),vp_3);
                cout<<"Press Q to proceed..."<<endl;
                (bVerbose == true) ? pg->spin() : pg->spinOnce();

                // Record correspondences
                wasp_registration_utils::CorrespondenceStructure new_correspondences(i,j, filtered_correspondences);
                all_correspondences.push_back(new_correspondences);
            } else {
                cout<<"Warning: the number of correspondences between clouds "<< i <<" and "<< j <<
                      " is "<< filtered_correspondences.size() << ". Skipping this pair of clouds." << endl;
            }
            pg->removeAllPointClouds(vp_1);
            pg->removeAllPointClouds(vp_2);
        }
    }

    /// save the correspondences to disk
    wasp_registration_utils::save_correspondences_to_file(clouds_folder+"/correspondences.txt",all_correspondences);

    return 0;
}
