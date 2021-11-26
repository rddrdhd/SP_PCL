//
// Created by rddrdhd on 26.11.21.
//

#ifndef MAIN_LOADER_H
#define MAIN_LOADER_H

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_from_file(std::string folderpath, std::string filepath);

#endif //MAIN_LOADER_H
