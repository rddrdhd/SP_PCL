//
// Created by rddrdhd on 8.5.22.
//

#ifndef MAIN_CLOUDER_H
#define MAIN_CLOUDER_H
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/console/time.h>
#include <pcl/keypoints/sift_keypoint.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

class Clouder {
public:
    Clouder();
    explicit Clouder(const char* filepath);
    void setCloud(pcl::PointCloud<PointType>::Ptr cloud){this->cloud_ = cloud;};
    unsigned long size(){return this->cloud_->size();};

    void showKeypoints();
    void showNormals();

    void computeNormals();
    void generateDownsampledCloud();
    void generateSIFTKeypoints();
    void generateFPFHDescriptors();
    pcl::PointCloud<PointType>::Ptr getCloud(){return this->cloud_;};
    pcl::PointCloud<PointType>::Ptr getKeypointsXYZ(){
        // Copying the pointwithscale to pointxyz so as visualize the cloud
        pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
        copyPointCloud(this->keypoints_, *cloud_temp);
        return cloud_temp;};
    pcl::PointCloud<PointType>::Ptr getDownsampledCloud(){return this->downsampled_cloud_;};
private:
    pcl::PointCloud<PointType>::Ptr cloud_;
    pcl::PointCloud<pcl::PointWithScale> keypoints_;
    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_; // Fast Point Feature Histogram descriptors
    pcl::PointCloud<PointType>::Ptr downsampled_cloud_;
    int k_neighbours_{10};// neighbours used to find normals
    float min_scale_{0.2f};//the standard deviation of the smallest scale in the scale space
    int n_octaves_{6};//the number of octaves (ie doublings of scale) to compute
    int n_scales_per_octave_{4};  //the number of scales to compute within each octave
    float min_contrast_{0.005f}; //the minimum contrast required for detection
    float downsampling_radius{0.1}; // spaces between points for uniform sampling
    int descriptor_k_neighbours_{15}; // k neighbours for SHOT descriptor

};


#endif //MAIN_CLOUDER_H
