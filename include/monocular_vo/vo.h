
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "vo_utils.h"
#include <fstream>
#include <iterator>
#include <vector>
#include <sstream>
#include <filesystem>

void saveAsGif(const std::string& filename, const std::vector<std::string>& frame_files, int delay) {
    std::string command = "python3 save_as_gif.py";
    for (const auto& file : frame_files) {
        command += " " + file;
    }
    command += " " + filename + " " + std::to_string(delay / 1000.0);
    system(command.c_str());
}
class VisualOdometry
{
    private:
     std::string dataset_path;
     std::string images_path;
     std::string calibrations_path;
     std::string poses_path;
    public :
      VisualOdometry(std::string dataset_path);
      void run(); 
};

VisualOdometry::VisualOdometry(std::string dataset_path)
{
    this->dataset_path = dataset_path;
    this->images_path = dataset_path + "/data_odometry_gray/dataset/sequences/00";
    this->calibrations_path = dataset_path + "/data_odometry_gray/dataset/sequences/00/calib.txt";
    this->poses_path = dataset_path + "/data_odometry_poses/dataset/poses/00.txt";
    std::cout << "Dataset path: " << dataset_path << std::endl;
}

void VisualOdometry::run()
{   
    const int MIN_NUM_FEAT = 1500;
    const int MAX_NUM_FRAME= 4541;
    cv::Mat prev_img, curr_img; // the previous and current images
    cv::Mat R_f, t_f; // the final rotation and translation vectors containing the cumulative transformation
    cv::Mat R_g , t_g; // the ground truth rotation and translation vectors
    std::vector<std::string> frame_files; // Vector to store frame file paths
    std::cout << "Running Visual Odometry" << std::endl;
    char image1[100];
    char image2[100];
    char text[100];
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 1;
    int thickness = 0.2;    
    cv::Point text_org(10, 50);
    sprintf(image1, "%s/image_0/%06d.png", images_path.c_str(), 0);
    sprintf(image2, "%s/image_0/%06d.png", images_path.c_str(), 1);
    //  read the images
    prev_img = cv::imread(image1, cv::IMREAD_GRAYSCALE);
    curr_img = cv::imread(image2, cv::IMREAD_GRAYSCALE);

    if (prev_img.empty() || curr_img.empty())
    {
        std::cout << "Could not read the images" << std::endl;
        return;
    }
    // feature detection,matching and tracking using FAST
    std::vector<cv::Point2f> points1 , points2;
    FeatureDetection(prev_img, points1);
    FeatureTracking(prev_img, curr_img, points1, points2);
    //get camera calibration parameters
    cv::Mat K ;
    getInternsicParam(calibrations_path, K);
    std::cout << "Camera matrix: " << K << std::endl;
    // Essential matrix
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(points2, points1, K, cv::RANSAC, 0.999, 1.0, mask);
    std::cout << "Essential matrix: " << E << std::endl;
    cv::recoverPose(E, points2, points1, K, R, t, mask);
    prev_img = curr_img.clone();
    std::vector<cv::Point2f> pre_feature = points2;
    std::vector<cv::Point2f> curr_feature;
    R_f = R.clone();
    t_f = t.clone();
    // Create a window for display.
    clock_t begin = clock();      // Start the timer
    namedWindow("Road facing Egomotion camera", cv::WINDOW_AUTOSIZE); // Camera Display Window
    namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);                   // Trajectory Display Window
    // Create a black image with the size of 600x600 
    cv::Mat traj = cv::Mat::zeros(800, 800, CV_8UC3);
    for (int frame_num= 2; frame_num < MAX_NUM_FRAME; frame_num++)
    {
        char image2[100];
        sprintf(image2, "%s/image_0/%06d.png", images_path.c_str(), frame_num);
        curr_img = cv::imread(image2, cv::IMREAD_GRAYSCALE);
        if (curr_img.empty())
        {
            std::cout << "Could not read the images" << std::endl;
            return;
        }
        FeatureTracking(prev_img, curr_img, pre_feature, curr_feature);
        // get essential matrix and recover the pose 
        E = cv::findEssentialMat(curr_feature, pre_feature, K, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, curr_feature, pre_feature, K, R, t, mask);
        double scale ; 
        scale = getAbsoluteScalePose(poses_path, frame_num); 
        if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
        {
            // t_final = t_previous + scale * (R_previous * t_current)
            t_f = t_f + scale * (R_f * t);
            R_f = R * R_f;  // R_final = R_current * R_previous
        }
        // a redetection is triggered in case the number of features being trakced go below a particular threshold
        if (pre_feature.size() < MIN_NUM_FEAT)
        {
            FeatureDetection(prev_img, pre_feature);
            FeatureTracking(prev_img, curr_img, pre_feature, curr_feature);
        } 
        prev_img = curr_img.clone();
        pre_feature = curr_feature;
        // ground truth 
        getGroundTruthPose(poses_path, frame_num , t_g , R_g);
        int x = int(t_f.at<double>(0)) + 330;      // offset for easier visualisation
        int y = int(-1 * t_f.at<double>(2)) + 550; // -1 inversion and offset for easier visualisation
        int x_g = int(t_g.at<double>(0)) + 330;      // offset for easier visualisation
        int y_g = int(-1 * t_g.at<double>(2)) + 550; // -1 inversion and offset for easier visualisation
        cv::circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 1);
        cv::circle(traj, cv::Point(x_g, y_g), 1, CV_RGB(0, 255, 0), 1);
        cv::rectangle(traj, cv::Point(10, 38), cv::Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);
        // sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
        // cv::putText(traj, text, text_org, font_face, font_scale, Scalar::all(255), thickness, 8);
        // resize the current image like the trajectory image
        cv::resize(curr_img, curr_img, cv::Size(500, 500));
        cv::imshow("Road facing Egomotion camera", curr_img );
        cv::imshow("Trajectory", traj);
        // add text legend red for estimated and green for ground truth
        cv::putText(traj, "Red: Estimated", cv::Point(10, 20), font_face, font_scale, cv::Scalar(0, 0, 255), thickness, 8);
        cv::putText(traj, "Green: Ground Truth", cv::Point(10,35), font_face, font_scale, cv::Scalar(0, 255,0 ), thickness, 8);
        cv::waitKey(1); 
        // Save the current frame
        if (frame_num % 5 == 0) { // Save every 5th frame to reduce the frame rate
            std::string frame_file = "frame_" + std::to_string(frame_num) + ".png";
            cv::imwrite(frame_file, traj);
            frame_files.push_back(frame_file);
        }
        // save the last picture 
        if (frame_num == MAX_NUM_FRAME - 1)
        {
            cv::imwrite("Road facing Egomotion camera.png", curr_img);
            cv::imwrite("Trajectory.png", traj);
            // saveAsGif("Trajectory.gif", frame_files, 100); // Save as GIF with 100ms delay between frames     
        }
        
    }
}




