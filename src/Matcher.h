//
// Created by rddrdhd on 21.5.22.
//

#ifndef MAIN_MATCHER_H
#define MAIN_MATCHER_H

#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/recognition/cg/geometric_consistency.h>
typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointNormal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 SHOTType;
typedef pcl::PFHSignature125 PFHType;
typedef pcl::FPFHSignature33 FPFHType;

class Matcher {
public:
    Matcher(pcl::PointCloud<PointType>::Ptr s,
        pcl::PointCloud<PointType>::Ptr m,
        pcl::PointCloud<PointType>::Ptr mk,
        pcl::PointCloud<PointType>::Ptr sk,
        pcl::CorrespondencesPtr c);

    pcl::PointCloud<PointType>::Ptr scene_;
    pcl::PointCloud<PointType>::Ptr model_;
    pcl::PointCloud<PointType>::Ptr model_keypoints_;
    pcl::PointCloud<PointType>::Ptr scene_keypoints_;
    pcl::CorrespondencesPtr correspondences_;
    void GCClustering(float cg_size = 0.01f, float cg_thresh = 5.0f);

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations_;
    std::vector<pcl::Correspondences> clustered_corrs_;
    void output();
};


#endif //MAIN_MATCHER_H
