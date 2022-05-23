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
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot_lrf.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointNormal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 SHOTType;
typedef pcl::PFHSignature125 PFHType;
typedef pcl::FPFHSignature33 FPFHType;

class Clouder {
public:
    Clouder();
    explicit Clouder(const char* filepath);

    void computeNormals(int k_neigh = 0, float r_neigh = 0.0);

    void generateDownsampledCloud(float downsampling_r = 0);
    void generateSIFTKeypoints( int octaves = 0, int scales = 0, float min_scale = 0, float min_contrast = 0);
    void generateHarrisKeypoints(float radius,float radius_search, int method_number = 1);

    void showKeypoints();
    void showNormals();

    void computeFPFHDescriptors(int k_neighbours = 0);
    void computePFHDescriptors();
    void computeSHOTDescriptors();

    void findKDTreeCorrespondencesFromSHOT(Clouder model);

    void setCloud(pcl::PointCloud<PointType>::Ptr cloud){this->cloud_ = cloud;};
    pcl::PointCloud<PointType>::Ptr getCloud(){return this->cloud_;};
    unsigned long size(){return this->cloud_->size();};

    void setKNormalNeigh(int k){this->k_normal_neighbours_ = k;}
    pcl::PointCloud<NormalType>::Ptr getNormals(){return this->point_normals_;};


    pcl::PointCloud<PointType>::Ptr getKeypointsXYZ();

    pcl::PointCloud<FPFHType>::Ptr FPFH_descriptors_; // Fast Point Feature Histogram descriptors
    pcl::PointCloud<PFHType>::Ptr PFH_descriptors_; // Point Feature Histogram descriptors
    pcl::PointCloud<SHOTType>::Ptr SHOT_descriptors_; // Signatures of Histograms of Orientations descriptors
    pcl::CorrespondencesPtr getCorrespondences(){return this->correspondences_;};
private:
    pcl::PointCloud<PointType>::Ptr cloud_;
    pcl::PointCloud<PointType>::Ptr downsampled_cloud_;

    pcl::PointCloud<NormalType>::Ptr point_normals_;
    int k_normal_neighbours_{5};// neighbours used to find normals

    pcl::PointCloud<pcl::PointWithScale> keypointsWithScale_;
    pcl::PointCloud<PointType> keypointsXYZ_;
    float min_scale_{0.2f}; // SIFT - the standard deviation of the smallest scale in the scale space
    int n_octaves_{6};  // SIFT - the number of octaves (ie doublings of scale) to compute
    int n_scales_per_octave_{4};    // SIFT - the number of scales to compute within each octave
    float min_contrast_{0.005f};    // SIFT - the minimum contrast required for detection

    float downsampling_radius_{0.015}; // spaces between points for uniform sampling
    int descriptor_k_neighbours_{15};   // FPFH - k neighbours (bigger than k_normal_neighbours_ for normals!)
    float descriptor_radius_{0.02f}; // SHOT - radius
    float rf_rad_{0.015f}; // SHOT - reference frame radius

    pcl::CorrespondencesPtr correspondences_;
};


#endif //MAIN_CLOUDER_H
