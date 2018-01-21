#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/filesystem.hpp>
#include <fstream>

#include "utils.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

namespace fs = ::boost::filesystem;
using namespace std;

void printUsage(){
    std::cout<<"Usage: load_and_view_data target_folder"<<std::endl;
    std::cout<<"Default folder: /home/wasp/wasp_registration/data/wasp_registration/"<<std::endl;
    std::cout<<"Note: this will look for all files matching *.pcd in the target_folder"<<std::endl;
}

int main(int argc, char** argv){
    std::string clouds_folder;
    if (argc < 2){
        printUsage();
        clouds_folder = "/home/wasp/wasp_registration/data/wasp_registration/"; // default data folder
    } else {
        clouds_folder = argv[1];
    }
    bool bVerbose = false; // if set to false the program will simply loop through the visualization steps, stopping briefly

    /****************************** LOAD AND VISUALIZE DATA ***************************************/
    if (!fs::is_directory(clouds_folder)){
        printUsage();
        exit(-1);
    }

    /****************************** VISUALIZE CLOUDS ***************************************/
    int vp_1, vp_2, vp_3;
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "visualize_clouds"); // initialize pcl visualizer
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

    return 0;
}
