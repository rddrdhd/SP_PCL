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

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/console/time.h>

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

    void computeNormals();
    void generateDownsampledCloud();
    void generateSIFTKeypoints();

    void showSHOTKeypoints();
    void showNormals();

    void computeFPFHDescriptors();
    void computePFHDescriptors();
    void computeSHOTDescriptors();

    pcl::PointCloud<PointType>::Ptr getCloud(){return this->cloud_;};
    pcl::PointCloud<PointType>::Ptr getKeypointsXYZ(){
        // Copying the pointwithscale to pointxyz so as visualize the cloud
        pcl::PointCloud<PointType>::Ptr cloud_temp (new pcl::PointCloud<PointType>);
        copyPointCloud(this->keypoints_, *cloud_temp);
        return cloud_temp;};
    pcl::PointCloud<PointType>::Ptr getDownsampledCloud(){return this->downsampled_cloud_;};
private:
    pcl::PointCloud<PointType>::Ptr cloud_;
    pcl::PointCloud<PointType>::Ptr downsampled_cloud_;

    pcl::PointCloud<pcl::PointNormal>::Ptr point_normals_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    int k_neighbours_{10};// neighbours used to find normals

    pcl::PointCloud<pcl::PointWithScale> keypoints_;
    float min_scale_{0.2f}; // SIFT - the standard deviation of the smallest scale in the scale space
    int n_octaves_{6};  // SIFT - the number of octaves (ie doublings of scale) to compute
    int n_scales_per_octave_{4};    // SIFT - the number of scales to compute within each octave
    float min_contrast_{0.005f};    // SIFT - the minimum contrast required for detection

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH_descriptors_; // Fast Point Feature Histogram descriptors
    pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH_descriptors_; // Point Feature Histogram descriptors
    pcl::PointCloud<pcl::SHOT352>::Ptr SHOT_descriptors_; // Signatures of Histograms of Orientations descriptors

    float downsampling_radius{0.1}; // spaces between points for uniform sampling
    int descriptor_k_neighbours_{15};   // FPFH - k neighbours (bigger than k_neighbours_!)
    float descriptor_radius_{0.1}; // SHOT - radius
    float rf_rad_{0.015f}; // SHOT - reference frame radius

};


#endif //MAIN_CLOUDER_H
