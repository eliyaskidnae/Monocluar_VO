
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <fstream>
#include <iterator>
#include <vector>
#include <sstream>
#include <filesystem>




void  getGroundTruthPose(std::string poses_path, int frame_num , cv::Mat &t_f , cv::Mat &R_f)
{
    std::ifstream file(poses_path);
    std::string line;
    double x, y, z;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    int i = 0;
    if (file.is_open())
    {
        while ((std::getline(file, line)) && (i <= frame_num))
        {
            std::istringstream iss(line);
            std::vector <std::string> results((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());
            x = std::stod(results[3]);
            y = std::stod(results[7]);
            z = std::stod(results[11]);
            R_f = (cv::Mat_<double>(3, 3) << std::stod(results[0]), std::stod(results[1]), std::stod(results[2]),
                 std::stod(results[4]), std::stod(results[5]), std::stod(results[6]),
                 std::stod(results[8]), std::stod(results[9]), std::stod(results[10]));
            t_f= (cv::Mat_<double>(3, 1) << x, y, z);


            i++;
        }
        file.close();
    }
    else
    {
        std::cout << "Could not open the file" << std::endl;
    }
    
}
double getAbsoluteScalePose(std::string poses_path, int frame_num)
{
    std::ifstream file(poses_path);
    std::string line;
    double  prev_x = 0 , prev_y = 0, prev_z = 0 , curr_x = 0, curr_y=0, curr_z=0;
    int i = 0;
    if (file.is_open()){ 
        while ( (std::getline(file , line )) && (i<=frame_num)){
            std::istringstream iss(line);
            std::vector <std::string> results((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());
            prev_x = curr_x;
            prev_y = curr_y;
            prev_z = curr_z;

            curr_x = std::stod(results[3]);
            curr_y = std::stod(results[7]);
            curr_z = std::stod(results[11]);
            
           
            i++;
        }
        file.close();

    }
    else
    {
        std::cout << "Could not open the file" << std::endl;
    }
    double dx = curr_x - prev_x;
    double dy = curr_y - prev_y;
    double dz = curr_z - prev_z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}
void getInternsicParam(std::string calibrations_path, cv::Mat &K)
{
    std::ifstream file(calibrations_path);
    std::string line;
    std::string key;
    double fx, fy, cx, cy;
    if (file.is_open())
    {
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::vector <std::string> results((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());
            key = results[0];
            fx = std::stod(results[1]);
            fy = std::stod(results[6]);
            cx = std::stod(results[3]);
            cy = std::stod(results[7]);
            K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            break;  // only first line is needed  
        }
    }
    else
    {
        std::cout << "Could not open the file" << std::endl;
    }

    file.close();
}
void FeatureTracking(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> &points1, std::vector<cv::Point2f> &points2)
{
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Size winSize = cv::Size(21, 21);
    cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // Calculate optical flow using Lucas-Kanade method
    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

    // Remove points for which tracking failed
    int indexCorrection = 0;
    for (size_t i = 0; i < status.size(); i++)
    {
        cv::Point2f pt = points2.at(i - indexCorrection);
        if (status.at(i) == 0)
        {
            points1.erase(points1.begin() + (i - indexCorrection));
            points2.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}
void FeatureDetection(cv::Mat img1, std::vector<cv::Point2f> &points1)
{
    std::vector<cv::KeyPoint> keypoints;   
    int fast_threshold = 30;
    bool nonmaxSuppression = true;
    cv::FAST(img1, keypoints, fast_threshold, nonmaxSuppression);
    cv::KeyPoint::convert(keypoints, points1 , std::vector<int>());
    
}