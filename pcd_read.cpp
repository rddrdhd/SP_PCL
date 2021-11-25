#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main ()
{
    //creates a PointCloud<PointXYZ> boost shared pointer and initializes it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    //loads the PointCloud data from file into the binary blob, works for binary/readable exported clouds
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../pcd_files/MODEL_cup_pink.pcd", *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (const auto& point: *cloud)
        std::cout << "    " << point.x
                  << " "    << point.y
                  << " "    << point.z << std::endl;
    return (0);
}