#ifndef __WASP_REGISTRATION_UTILS_HH
#define __WASP_REGISTRATION_UTILS_HH

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/features2d.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>

namespace wasp_registration_utils{

    /** Method which loads point clouds from a folder
     * Note that this returns the data after applying a sort operation on the file names (i.e. cloud000 < cloud001 < ... < cloud010 < ...
     * (Importantly, this also corresponds to the order in which the clouds were acquired by the robot).
     *
     * @param folder the folder on the disk from where the point clouds should be loaded.
     * @return a vector containing the Point Cloud objects.
     */
    template<class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> load_pcds(const std::string& folder){
        // This function will look for *.pcd files withing a folder
        namespace fs = ::boost::filesystem;
        std::vector<std::string> pcd_files;
        fs::recursive_directory_iterator it(folder);
        fs::recursive_directory_iterator endit;

        while(it != endit) {
            if(fs::is_regular_file(*it) && it->path().extension() == ".pcd") {
                pcd_files.push_back(it->path().filename().string());
            }
            ++it;
        }

        std::sort(pcd_files.begin(), pcd_files.end());

        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_pcds;
        pcl::PCDReader reader;
        for (int i=0; i<pcd_files.size(); ++i){
            boost::shared_ptr<pcl::PointCloud<PointType>> cloud (new pcl::PointCloud<PointType>);
            cout<<"Loading "<<pcd_files[i]<<endl;
            reader.read (folder + "/" + pcd_files[i], *cloud);
            all_pcds.push_back(cloud);
        }

        return all_pcds;
    }


    /** Method which downsamples a point cloud
     *
     * @param input - the input point cloud
     * @param x_size - the size along the x dimension of the voxel grid used for downsampling
     * @param y_size - the size along the y dimension of the voxel grid used for downsampling
     * @param z_size - the size along the z dimension of the voxel grid used for downsampling
     * @return the downsampled point cloud
     */
    template<class PointType>
    boost::shared_ptr<pcl::PointCloud<PointType>> downsample_cloud(boost::shared_ptr<pcl::PointCloud<PointType>> input,
                                                                   const double& x_size = 0.05,
                                                                   const double& y_size = 0.05,
                                                                   const double& z_size = 0.05){

        boost::shared_ptr<pcl::PointCloud<PointType>> output ( new pcl::PointCloud<PointType>);

        /** ----------------- YOUR CODE HERE -------------------------
         * Add your code here to downsaple a point cloud
         */

        return output;
    }

    /** Method which downsamples and also removes NAN values from a point cloud
     *
     * @param input - the input point cloud
     * @param x_size - the size along the x dimension of the voxel grid used for downsampling
     * @param y_size - the size along the y dimension of the voxel grid used for downsampling
     * @param z_size - the size along the z dimension of the voxel grid used for downsampling
     * @return the downsampled point cloud
     */
    template<class PointType>
    boost::shared_ptr<pcl::PointCloud<PointType>> downsample_and_filter_cloud(boost::shared_ptr<pcl::PointCloud<PointType>> input,
                                                                   const double& x_size = 0.05,
                                                                   const double& y_size = 0.05,
                                                                   const double& z_size = 0.05){

        boost::shared_ptr<pcl::PointCloud<PointType>> output = downsample_cloud<PointType>(input, x_size, y_size, z_size);

        /** ----------------- YOUR CODE HERE -------------------------
         * Add your code here to remove points with invalid coordinates from the point cloud
         */

        return output;
    }

    /**
     * @brief create_RGB_and_depth_from_cloud -
     * @param cloud - input point cloud
     * @return a pair of cv images, the first one corresponding to the RGB image (type CV_8UC3), and the second one to the depth image (type CV_16UC1).
     */
    std::pair<cv::Mat, cv::Mat> create_RGB_and_depth_from_cloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud);

    /**
     * @brief get_pcd_index_from_image_keypoint - method which returns the index of a point in a point cloud which corresponds to a cv::Mat keypoint
     * @param image - cv image
     * @param kp - cv image keypoint
     * @return index in the point cloud which corresponds to the keypoint in the cv image
     */
    int get_pcd_index_from_image_keypoint(const cv::Mat& image, const cv::KeyPoint& kp);

    /**
     * @brief get_image_keypoint_from_pcd_index
     * @param image
     * @param index
     * @return
     */
    cv::KeyPoint get_image_keypoint_from_pcd_index(const cv::Mat& image, const int& index);


    /**
     * @brief The CorrespondenceStructure struct - used to record PCL correspondences, as well as the indices of the point clouds which generated the correspondences
     * @param idx1 - the index of the first point cloud
     * @param idx2 - the index of the second point cloud
     */
    struct CorrespondenceStructure{
        int idx1;
        int idx2;
        pcl::Correspondences correspondences;

        CorrespondenceStructure(const int& i1, const int& i2, const pcl::Correspondences& c){
            idx1 = i1;
            idx2 = i2;
            correspondences.assign(c.begin(),c.end());
        }

        CorrespondenceStructure() : idx1(-1), idx2(-1){

        }
    };

    /**
     * @brief save_correspondences_to_file - method which will save a set of correspondences to a file
     * @param filename - the name of the file where to save the correspondences
     * @param all_correspondences - the structure containing sets of correspondences between different point clouds
     */
    void save_correspondences_to_file(const std::string& filename,
                                      const std::vector<CorrespondenceStructure>& all_correspondences);


    /**
     * @brief load_correspondences_from_file - method which will load a set from a file
     * @param filename - name of the file from where to load the correspondences
     * @return the correspondences
     */
    std::vector<CorrespondenceStructure>
    load_correspondences_from_file(const std::string& filename);

    /**
     * @brief visualize_correspondences - method which will use opencv to draw correspondences on two images
     * @param img1 - the first image
     * @param img2 - the second image
     * @param correspondences - the correspondences (in PCL format)
     * @param window_name - the name of the window where to display the images and the correspondences
     * @param display_timeout - how long to wait until resuming the program. If 0 the program will wait for a keypress
     */
    void
    visualize_correspondences(const cv::Mat& img1, const cv::Mat& img2,
                              const pcl::Correspondences& correspondences,
                              const std::string& window_name,
                              const int& display_timeout);
}

#endif


