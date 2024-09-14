
#include <iostream>
#include <string>
#include <ros/package.h>
#include <monocular_vo/vo.h>


int main(int argc,  char** argv) {
    // ros package path 
    std::string package_path = ros::package::getPath("monocular_vo");

    std::string dataset_path = package_path + "/kitti_dataset" ;
    VisualOdometry visual_odom(dataset_path);
    return 0;
}

