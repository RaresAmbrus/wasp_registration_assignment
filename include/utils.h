#ifndef __WASP_REGISTRATION_UTILS_HH
#define __WASP_REGISTRATION_UTILS_HH

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
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
        for (int i=0; i<pcd_files.size(); ++i){
            pcl::PCDReader reader;
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

}
#endif


