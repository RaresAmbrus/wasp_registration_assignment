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

    template<class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> load_pcds(const std::string& folder){
        // This function will look for *.pcd files withing a folder
        namespace fs = ::boost::filesystem;
        std::vector<std::string> pcd_files;
        fs::recursive_directory_iterator it(folder);
        fs::recursive_directory_iterator endit;
        const boost::regex my_filter( ".*.pcd" );
        boost::smatch what;

        while(it != endit)
        {
            if(fs::is_regular_file(*it) && boost::regex_match( it->path().string(), what, my_filter )) pcd_files.push_back(it->path().filename().string());
            ++it;
        }

        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> all_pcds;
        for (int i=0; i<pcd_files.size(); ++i){
            pcl::PCDReader reader;
            boost::shared_ptr<pcl::PointCloud<PointType>> cloud (new pcl::PointCloud<PointType>);
            std::stringstream ss_name;
            if (i<10){
                ss_name<<"cloud_00";
            } else {
                ss_name<<"cloud_0";
            }
            ss_name<<i<<".pcd";
            reader.read (folder + "/" + ss_name.str(), *cloud);
            all_pcds.push_back(cloud);
        }

        return all_pcds;
    }   

}
#endif


