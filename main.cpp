#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Get shared pointer to a cloud from file
pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_from_file(std::string filepath){

    //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //loads the PointCloud data from file into the binary blob, works for binary/readable exported clouds
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filepath, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
    return cloud;
}
int main ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = get_cloud_from_file("../pcd_files/MODEL_cup_pink.pcd");
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (const auto& point: *cloud)
        std::cout << "\t" << point.x
                  << "\t" << point.y
                  << "\t" << point.z << std::endl;
    return (0);
}