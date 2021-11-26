//
// Created by rddrdhd on 26.11.21.
//

#include "loader.h"

// Get shared pointer to a cloud from file
pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_from_file(std::string folderpath, std::string filepath){

    //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //loads the PointCloud data from file into the binary blob, works for binary/readable exported clouds
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ((folderpath+filepath).c_str(), *cloud) == -1)
    {
        auto msg = "Couldn't read file "+filepath+" \n";
        PCL_ERROR (msg.c_str());
    }
    return cloud;
}
