#include "utils.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

std::pair<cv::Mat, cv::Mat> wasp_registration_utils::create_RGB_and_depth_from_cloud(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud)
{
    std::pair<cv::Mat, cv::Mat> toRet;
    toRet.first = cv::Mat::zeros(480, 640, CV_8UC3); // RGB image
    toRet.second = cv::Mat::zeros(480, 640, CV_16UC1); // Depth image
    pcl::PointXYZRGB point;
    for (size_t y = 0; y < toRet.first.rows; ++y) {
        for (size_t x = 0; x < toRet.first.cols; ++x) {
            point = cloud->points[y*toRet.first.cols + x];
            // RGB
            toRet.first.at<cv::Vec3b>(y, x)[0] = point.b;
            toRet.first.at<cv::Vec3b>(y, x)[1] = point.g;
            toRet.first.at<cv::Vec3b>(y, x)[2] = point.r;
            // Depth
            if (!pcl::isFinite(point)) {
                toRet.second.at<u_int16_t>(y, x) = 0.0; // convert to uint 16 from meters
            } else {
                toRet.second.at<u_int16_t>(y, x) = point.z*1000; // convert to uint 16 from meters
            }
        }
    }
    return toRet;
}

int wasp_registration_utils::get_pcd_index_from_image_keypoint(const cv::Mat& image, const cv::KeyPoint& kp){
    int point_index = (int)kp.pt.y * image.cols + (int)kp.pt.x;
    return point_index;
}

cv::KeyPoint wasp_registration_utils::get_image_keypoint_from_pcd_index(const cv::Mat& image, const int& index){
    cv::KeyPoint kp (index % image.cols,
                     index / image.cols, 1.0);
    return kp;
}

void wasp_registration_utils::save_correspondences_to_file(const std::string& filename,
                                                           const std::vector<wasp_registration_utils::CorrespondenceStructure>& all_correspondences){
    std::ofstream out;
    out.open(filename.c_str());
    std::cout<<"Saving correspondences to "<<filename<<endl;

    out<<all_correspondences.size()<<endl;
    for (int i=0; i< all_correspondences.size(); ++i){
        out<<all_correspondences[i].idx1<<" "<<all_correspondences[i].idx2<<" "<<all_correspondences[i].correspondences.size()<<std::endl;

        for (int j=0; j<all_correspondences[i].correspondences.size(); ++j){
            pcl::Correspondence c = all_correspondences[i].correspondences[j];
            out<<c.index_query<<" "<<c.index_match<<std::endl;
        }

        out<<std::endl;
    }
    out.close();
}

std::vector<wasp_registration_utils::CorrespondenceStructure>
wasp_registration_utils::load_correspondences_from_file(const std::string& filename){

    std::vector<wasp_registration_utils::CorrespondenceStructure> all_correspodences;
    ifstream in;
    in.open(filename.c_str());

    int all_c_size;
    in>>all_c_size;

    for (int i=0; i<all_c_size; ++i){
        wasp_registration_utils::CorrespondenceStructure new_correspondences;
        int c_count;
        in >> new_correspondences.idx1 >> new_correspondences.idx2 >> c_count;

        for (int j=0; j<c_count; ++j){
            pcl::Correspondence c;
            in >> c.index_query >> c.index_match;
            new_correspondences.correspondences.push_back(c);
        }

        all_correspodences.push_back(new_correspondences);
    }

    return all_correspodences;
}

void
wasp_registration_utils::visualize_correspondences(const cv::Mat& img1, const cv::Mat& img2,
                               const pcl::Correspondences& correspondences,
                               const std::string& window_name,
                               const int& display_timeout){
    // convert correspondences to opencv datastructure X-(
    std::vector< cv::DMatch > cv_correspondences;
    std::vector<cv::KeyPoint> cv_keypoints_1, cv_keypoints_2;

    for (size_t j=0; j<correspondences.size(); ++j){
        pcl::Correspondence c = correspondences[j];

        // Convert to cv KeyPoints for plotting
        cv::KeyPoint k1 (c.index_query % img1.cols,
                         c.index_query / img1.cols, 1.0);
        cv_keypoints_1.push_back(k1);

        cv::KeyPoint k2 (c.index_match % img2.cols,
                         c.index_match / img2.cols, 1.0);
        cv_keypoints_2.push_back(k2);

        cv::DMatch mm(j,j,1.0);
        cv_correspondences.push_back(mm);
    }

    // Plot the correspondences
    cv::Mat matches_img;
    drawMatches( img1, cv_keypoints_1, img2, cv_keypoints_2,
                 cv_correspondences, matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    cv::resize(matches_img, matches_img, cv::Size(640, 240));
    // Show image
    cv::imshow( window_name.c_str(), matches_img );
    cv::waitKey(display_timeout);

}
