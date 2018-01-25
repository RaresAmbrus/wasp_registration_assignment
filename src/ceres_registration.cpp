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

#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>

#include <pcl/registration/icp.h>
#include <ceres/ceres.h>

#include <fstream>
#include "utils.h"
#include "simple_residual.h"
#include "scaled_residual.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

namespace fs = ::boost::filesystem;
using namespace std;
using namespace cv;
using namespace ceres;

void printUsage(){
    std::cout<<"Usage: ceres_registration target_folder correspondence_file"<<std::endl;
    std::cout<<"Default folder: /home/wasp/wasp_registration/data/wasp_registration/"<<std::endl;
    std::cout<<"Default correspondence file: target_folder/wasp_correspondences.txt"<<std::endl;
    std::cout<<"Note: this will look for all files matching *.pcd in the target_folder"<<std::endl;
}

int main(int argc, char** argv){
    std::string clouds_folder;
    std::string correspondences_file;
    if (argc == 3){
        clouds_folder = argv[1];
        correspondences_file = argv[2];
    } else {
        printUsage();
        clouds_folder = "/home/wasp/code/wasp_registration/data/wasp_registration/";
        correspondences_file = "correspondences.txt";
        if (argc==2){
            clouds_folder = argv[1];
            correspondences_file = "correspondences.txt";
        }
    }

    bool bVerbose = false;
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


    // Load the saved correspondences from file
    vector<wasp_registration_utils::CorrespondenceStructure> all_correspondences;
    all_correspondences = wasp_registration_utils::load_correspondences_from_file(clouds_folder + "/" + correspondences_file);

    cout<<"Visualizing correspondences."<<endl;
    for (auto c: all_correspondences){
        cv::Mat rgb1 = wasp_registration_utils::create_RGB_and_depth_from_cloud(all_views[c.idx1]).first;
        cv::Mat rgb2 = wasp_registration_utils::create_RGB_and_depth_from_cloud(all_views[c.idx2]).first;
        wasp_registration_utils::visualize_correspondences(rgb1, rgb2, c.correspondences, "test", 200);
    }

    // Set up optimization problem
    /**
     * @brief The Camera struct is used to store the transform of each point cloud.
     * It consist of a quaternion and a translation object which will be passed to Ceres during the optimization.
     */
    struct Camera{
        double quaternion[4] = {1.0,0.0,0.0,0.0};
        double translation[3] = {0.0,0.0,0.0};
        Camera(){};
    };

    /**
     * @brief Typical Ceres boilerplate code. Define a problem object, the solver type and the tolerances.
     *
     */
    Problem problem;
    Solver::Options options;
    options.function_tolerance = 1e-30;
    options.parameter_tolerance = 1e-20;
    options.max_num_iterations = 1000;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 2;
    options.num_linear_solver_threads = 2;

    /**
     * @brief all_cameras - this vector stores the transformations for all the point clouds, which are initialized as identity.
     */
    vector<Camera*>  all_cameras;
    // initial solution
    for (size_t i=0; i<all_views.size(); i++){
        Camera* cam = new Camera();
        all_cameras.push_back(cam);
        problem.AddParameterBlock(cam->quaternion,4);
        problem.AddParameterBlock(cam->translation,3);
    }


    /**
      * We will iterate through each set of correspondences. Be sure to check the definition of the CorrespondenceStructure structure to see how to access the data.
      */
    for (auto c: all_correspondences){
        Camera* cam1 = all_cameras[c.idx1];
        Camera* cam2 = all_cameras[c.idx2];

        /** ----------------- 1. YOUR CODE HERE -------------------------
         * You should iterate through each correspondence (i.e. each pair of points) for the two current camera objects.
         * For each correspondence, extract the 3D point from the original point clouds. For this you will have to use the index_query
         * and index_match fields. Recall that the original point clouds are stored in the all_views vector.
         * For each of the two 3D points, create corresponding C-style arrays of doubles (see constructor of SimpleResidual).
         */




        /** ----------------- 2. YOUR CODE HERE -------------------------
         * Define a CostFunction using the SimpleResidual class. Make sure to fill in the relevant code in the SimpleResidual class.
         * Define a LossFunction class associated to your CostFunction. You can use the HuberLoss with a weight of 1.
         * Use the problem.AddResidualBlock method to add your new CostFunction and Loss function to the optimization. Be sure to include the appropriate quaternion and translation elements.
         */

        /** ----------------- 3. YOUR CODE HERE -------------------------
         * Recall that in Part III you generated correspondences between pairs of clouds (i,i+1). In some cases, this is usually not enough to constrain the optimization.
         * Go back to your code from Part III and modify the method get_corresponding_IDXs_for_current_IDx. The default code returns a vector with a single element.
         * Modify that function so that, given index i, you return {i-2, i-1, i+1, i+2}; be sure to check that those values are valid (i.e. 0<=i-2<=size of cloud vector).
         * Generate and save the new correspondences.
         * Re-run your code and inspect the results. The additional correspondences should help constrain the optimization.
         */


        /** ----------------- 4. YOUR CODE HERE -------------------------
          * Incorporate the RGBD camera noise model into the optimization.
          * For each pair of 3D points, compute the weight as described in the assignment document.
          * Define a new residual class - ScaledResidual, which incorporates this weight. Basically, your code should be almost identical to
          * SimpleResidual, but you will add another parameter to the constructor of the class (i.e. the weight), and use it to scale the value of the
          * residual before returning it.
          * Add a LossFunction as before, and pass the appropriate weight this time.
          * Note: your new CostFunction and LossFunction should replace the ones you defined at point 2 above.
          * Re-run the optimization: your point clouds should be well registered now!
          */

    }

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    if (bVerbose){
        cout << summary.FullReport() << endl;
    }

    for (int i=0; i<all_cameras.size(); ++i){
        Eigen::Quaternionf q(all_cameras[i]->quaternion[0], all_cameras[i]->quaternion[1],all_cameras[i]->quaternion[2],all_cameras[i]->quaternion[3]);
        Eigen::Vector3f t(all_cameras[i]->translation[0], all_cameras[i]->translation[1], all_cameras[i]->translation[2]);

        Eigen::Matrix4f trans; trans.setIdentity();
        trans.block<3,1>(0,3) = t;
        trans.block<3,3>(0,0) =  q.normalized().toRotationMatrix();
        CloudPtr transformed(new Cloud);
        pcl::transformPointCloud(*all_views[i], *transformed, trans);

        stringstream ss; ss<<"Cloud";ss<<i;
        cout<<"Displaying point cloud "<<i<<". Press Q to continue."<<endl;
//        pg->addPointCloud(transformed, ss.str()); // uncomment this to see the full resolution point cloud (might slow things down considerably).
        pg->addPointCloud(wasp_registration_utils::downsample_cloud<PointType>(transformed,0.03, 0.03, 0.03), ss.str());

        (bVerbose == true) ? pg->spin() : pg->spinOnce();
    }
    pg->spin();

    return 0;
}
